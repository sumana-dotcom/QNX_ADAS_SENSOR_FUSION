/*
 * QNX SDP 8 / Raspberry Pi 4 — ADAS Sensor-Fusion Prototype
 *
 * Hardware:
 *   HC-SR04-style ultrasonic: TRIG GPIO23, ECHO GPIO24
 *   Digital IR obstacle sensor: GPIO25 (LOW = obstacle)
 *   Warning LED: GPIO18
 *
 * Threads: ultrasonic, IR, fusion, watchdog, logger, LED + main.
 * This is an educational prototype, NOT a safety-certified ADAS system.
 *
 * Hardware safety: Raspberry Pi GPIO is 3.3 V only. HC-SR04 ECHO is
 * commonly 5 V; use a level shifter or voltage divider before GPIO24.
 * Verify physical addresses, permissions, and GPIO setup for your QNX image.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/types.h>
#include <unistd.h>

/* BCM2711 GPIO physical address and register offsets. */
#define BCM2711_PERI_BASE       0xFE000000ULL
#define GPIO_BASE              (BCM2711_PERI_BASE + 0x200000ULL)
#define GPIO_MAP_LEN            4096
#define TRIG_PIN                23
#define ECHO_PIN                24
#define IR_PIN                  25
#define LED_PIN                 18
#define REG_GPSET0              0x1C
#define REG_GPCLR0              0x28
#define REG_GPLEV0              0x34

/* Sampling periods, filter lengths, and fault thresholds. */
#define ULTRA_WINDOW            5
#define IR_WINDOW               4
#define ULTRA_INTERVAL_MS       60
#define IR_INTERVAL_MS          5
#define FUSION_INTERVAL_MS      200
#define WATCHDOG_INTERVAL_MS    100
#define ECHO_TIMEOUT_NS         30000000ULL
#define STALE_TIMEOUT_NS        1000000000ULL
#define ULTRA_STUCK_NS          10000000000ULL
#define IR_STUCK_NS             30000000000ULL
#define MAX_ULTRA_ERRORS        3
#define ULTRA_CHANGE_CM         0.3
#define IR_NEAR_CM              10.0
#define IR_FAR_CM               30.0
#define ULTRA_TASK_TIMEOUT_NS   500000000ULL
#define IR_TASK_TIMEOUT_NS      500000000ULL
#define FUSION_TASK_TIMEOUT_NS  1000000000ULL
#define EVENT_QUEUE_SIZE        32
#define EVENT_TEXT_SIZE         192
#define FAULT_LOG_PATH          "/tmp/adas_fault.log"

/* The system is either operating normally or reporting reduced confidence. */
typedef enum {
    MODE_NORMAL = 0,
    MODE_DEGRADED = 1
} AdasMode;

/*
 * Shared sensor/task state. Every access to this structure must be protected
 * by data_mutex. Heartbeats indicate software progress, not sensor accuracy.
 */
typedef struct {
    double distance_cm;
    int ir_obstacle;
    int ultrasonic_valid;
    int ir_valid;
    int ultrasonic_errors;
    int ir_errors;
    uint64_t ultra_last_good;
    uint64_t ir_last_good;
    uint64_t ultra_last_change;
    uint64_t ir_last_change;
    double ultra_last_value;
    int ir_last_value;
    int ultra_stuck;
    int ir_stuck;
    uint64_t ultra_heartbeat;
    uint64_t ir_heartbeat;
    uint64_t fusion_heartbeat;
    int ultra_task_fault;
    int ir_task_fault;
    int fusion_task_fault;
    AdasMode mode;
} SensorData;

/* One record in the asynchronous fault-log queue. */
typedef struct {
    char text[EVENT_TEXT_SIZE];
} FaultEvent;

static volatile uint32_t *gpio_regs = NULL;
static SensorData shared = {0};
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Producer-consumer queue: sensor threads enqueue; logger writes to disk. */
static pthread_mutex_t event_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t event_cond = PTHREAD_COND_INITIALIZER;
static FaultEvent event_queue[EVENT_QUEUE_SIZE];
static int event_head = 0;
static int event_tail = 0;
static int event_count = 0;
static unsigned int dropped_events = 0;

/* Return monotonic time in nanoseconds for elapsed-time measurements. */
static uint64_t now_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0;

    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* Sleep for a number of milliseconds; resume if interrupted by a signal. */
static void sleep_ms(unsigned int ms)
{
    struct timespec ts;

    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;

    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
        /* nanosleep updates ts with the remaining time. */
    }
}

/*
 * Short busy-wait used for the ultrasonic trigger pulse.
 * Busy-waiting consumes CPU; echo polling below is another known limitation.
 */
static void busy_wait_us(unsigned int us)
{
    uint64_t start = now_ns();
    uint64_t duration = (uint64_t)us * 1000ULL;

    while (now_ns() - start < duration) {
        /* Wait until the requested interval has elapsed. */
    }
}

/*
 * Enqueue a timestamped event without doing file I/O in the caller.
 * If the bounded queue is full, count the dropped event and return.
 */
static void log_event(const char *message)
{
    uint64_t t;
    FaultEvent *slot;

    if (message == NULL)
        return;

    t = now_ns();
    pthread_mutex_lock(&event_mutex);

    if (event_count >= EVENT_QUEUE_SIZE) {
        dropped_events++;
        pthread_mutex_unlock(&event_mutex);
        return;
    }

    slot = &event_queue[event_tail];
    (void)snprintf(slot->text, sizeof(slot->text),
                   "[mono %llu.%03llu] %s",
                   (unsigned long long)(t / 1000000000ULL),
                   (unsigned long long)((t / 1000000ULL) % 1000ULL),
                   message);

    event_tail = (event_tail + 1) % EVENT_QUEUE_SIZE;
    event_count++;
    pthread_cond_signal(&event_cond);
    pthread_mutex_unlock(&event_mutex);
}

/*
 * Logger thread: wait for queued events, remove one under the queue mutex,
 * then release the mutex before opening/writing the log file.
 */
static void *logger_thread(void *arg)
{
    (void)arg;

    for (;;) {
        FaultEvent event;
        FILE *fp;

        pthread_mutex_lock(&event_mutex);
        while (event_count == 0)
            pthread_cond_wait(&event_cond, &event_mutex);

        event = event_queue[event_head];
        event_head = (event_head + 1) % EVENT_QUEUE_SIZE;
        event_count--;
        pthread_mutex_unlock(&event_mutex);

        fp = fopen(FAULT_LOG_PATH, "a");
        if (fp != NULL) {
            (void)fprintf(fp, "%s\n", event.text);
            (void)fclose(fp);
        }
        /* Continue monitoring if the log path cannot be opened. */
    }

    return NULL;
}

/* Configure a BCM GPIO pin: function 0=input, function 1=output. */
static void set_pin_function(int pin, unsigned int function)
{
    unsigned int reg = (unsigned int)(pin / 10);
    unsigned int shift = (unsigned int)((pin % 10) * 3);
    uint32_t value = gpio_regs[reg];

    value &= ~(7U << shift);
    value |= ((function & 7U) << shift);
    gpio_regs[reg] = value;
}

/* Drive GPIO pin HIGH or LOW. This helper supports GPIO0 through GPIO31. */
static void gpio_write(int pin, int high)
{
    if (gpio_regs == NULL || pin < 0 || pin > 31)
        return;

    if (high)
        gpio_regs[REG_GPSET0 / 4] = 1U << pin;
    else
        gpio_regs[REG_GPCLR0 / 4] = 1U << pin;
}

/* Read GPIO logic level (0 or 1); invalid access returns 0 in this prototype. */
static int gpio_read(int pin)
{
    if (gpio_regs == NULL || pin < 0 || pin > 31)
        return 0;

    return (gpio_regs[REG_GPLEV0 / 4] & (1U << pin)) ? 1 : 0;
}

/*
 * Request I/O privilege and map the GPIO registers into this process.
 * ThreadCtl/mmap can fail if QNX permissions or the image configuration
 * do not allow direct physical GPIO access.
 */
static int init_gpio(void)
{
    void *mapped;

    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("ThreadCtl failed");
        return -1;
    }

    mapped = mmap(NULL, GPIO_MAP_LEN,
                  PROT_READ | PROT_WRITE | PROT_NOCACHE,
                  MAP_SHARED | MAP_PHYS, NOFD, (off_t)GPIO_BASE);

    if (mapped == MAP_FAILED) {
        perror("GPIO mmap failed");
        return -1;
    }

    gpio_regs = (volatile uint32_t *)mapped;

    set_pin_function(TRIG_PIN, 1);
    set_pin_function(ECHO_PIN, 0);
    set_pin_function(IR_PIN, 0);
    set_pin_function(LED_PIN, 1);
    gpio_write(TRIG_PIN, 0);
    gpio_write(LED_PIN, 0);
    return 0;
}

/* Poll until ECHO reaches the requested level, or time out. */
static int wait_for_echo(int level, uint64_t timeout_ns)
{
    uint64_t start = now_ns();

    while (gpio_read(ECHO_PIN) != level) {
        uint64_t current = now_ns();

        if (current == 0 || current - start >= timeout_ns)
            return -1;
    }

    return 0;
}

/*
 * Trigger the HC-SR04-style sensor and measure ECHO high-pulse duration.
 * Distance (cm) ~= pulse duration (ns) * 0.00001715.
 * Return 0 on a valid 2..400 cm measurement, otherwise -1.
 */
static int read_ultrasonic(double *distance_cm)
{
    uint64_t start, end;

    if (distance_cm == NULL || gpio_regs == NULL)
        return -1;

    gpio_write(TRIG_PIN, 0);
    busy_wait_us(2);
    gpio_write(TRIG_PIN, 1);
    busy_wait_us(10);
    gpio_write(TRIG_PIN, 0);

    if (wait_for_echo(1, ECHO_TIMEOUT_NS) != 0)
        return -1;

    start = now_ns();
    if (wait_for_echo(0, ECHO_TIMEOUT_NS) != 0)
        return -1;

    end = now_ns();
    if (end <= start)
        return -1;

    *distance_cm = (double)(end - start) * 0.00001715;
    if (*distance_cm < 2.0 || *distance_cm > 400.0)
        return -1;

    return 0;
}

/* Thread 1: read ultrasonic distance and publish a five-sample average. */
static void *ultrasonic_thread(void *arg)
{
    double samples[ULTRA_WINDOW] = {0};
    int count = 0, index = 0;
    (void)arg;

    for (;;) {
        double distance;
        uint64_t now;
        int read_ok = (read_ultrasonic(&distance) == 0);

        now = now_ns();
        pthread_mutex_lock(&data_mutex);

        if (read_ok) {
            double sum = 0.0, average, delta;
            int i;

            samples[index] = distance;
            index = (index + 1) % ULTRA_WINDOW;
            if (count < ULTRA_WINDOW)
                count++;

            for (i = 0; i < count; i++)
                sum += samples[i];

            average = sum / (double)count;
            delta = average - shared.ultra_last_value;
            if (delta < 0.0)
                delta = -delta;

            /* Track last significant change for stuck-value suspicion. */
            if (shared.ultra_last_good == 0 || delta >= ULTRA_CHANGE_CM)
                shared.ultra_last_change = now;

            shared.ultra_last_value = average;
            shared.distance_cm = average;
            shared.ultra_last_good = now;
            shared.ultrasonic_valid = 1;
            shared.ultrasonic_errors = 0;
        } else {
            shared.ultrasonic_errors++;
            if (shared.ultrasonic_errors >= MAX_ULTRA_ERRORS)
                shared.ultrasonic_valid = 0;
        }

        shared.ultra_heartbeat = now_ns();
        pthread_mutex_unlock(&data_mutex);
        sleep_ms(ULTRA_INTERVAL_MS);
    }

    return NULL;
}

/* Thread 2: sample digital IR output and apply a majority-vote filter. */
static void *ir_thread(void *arg)
{
    int samples[IR_WINDOW] = {0};
    int count = 0, index = 0;
    (void)arg;

    for (;;) {
        int obstacle = (gpio_read(IR_PIN) == 0) ? 1 : 0; /* LOW = hit */
        uint64_t now = now_ns();
        int sum = 0, filtered, i;

        samples[index] = obstacle;
        index = (index + 1) % IR_WINDOW;
        if (count < IR_WINDOW)
            count++;

        for (i = 0; i < count; i++)
            sum += samples[i];

        filtered = (sum > count / 2);

        pthread_mutex_lock(&data_mutex);
        if (shared.ir_last_good == 0 ||
            filtered != shared.ir_last_value)
            shared.ir_last_change = now;

        shared.ir_last_value = filtered;
        shared.ir_obstacle = filtered;
        shared.ir_last_good = now;
        shared.ir_valid = 1;
        shared.ir_errors = 0;
        shared.ir_heartbeat = now;
        pthread_mutex_unlock(&data_mutex);

        sleep_ms(IR_INTERVAL_MS);
    }

    return NULL;
}

/* Change operating mode and log only actual transitions. */
static void set_mode(AdasMode new_mode)
{
    int changed = 0;

    pthread_mutex_lock(&data_mutex);
    if (shared.mode != new_mode) {
        shared.mode = new_mode;
        changed = 1;
    }
    pthread_mutex_unlock(&data_mutex);

    if (changed) {
        if (new_mode == MODE_DEGRADED) {
            printf("ADAS MODE: DEGRADED\n");
            log_event("ADAS_MODE_CHANGED: DEGRADED");
        } else {
            printf("ADAS MODE: NORMAL\n");
            log_event("ADAS_MODE_CHANGED: NORMAL");
        }
        fflush(stdout);
    }
}

/*
 * Thread 3: fusion loop.
 * - Marks stale readings invalid.
 * - Detects sensor faults and disagreements.
 * - Logs transitions rather than flooding the log on every cycle.
 * - Suppresses OBSTACLE/ROAD CLEAR output while degraded.
 */
static void *fusion_thread(void *arg)
{
    int first_result = 1;
    int previous_obstacle = 0;
    int previous_ultra_fault = 0, previous_ir_fault = 0;
    int previous_disagreement = 0;
    int previous_ultra_stuck = 0, previous_ir_stuck = 0;
    AdasMode previous_mode = MODE_DEGRADED;
    (void)arg;

    for (;;) {
        SensorData s;
        uint64_t now = now_ns();
        int ultra_fault, ir_fault, disagreement = 0;
        int degraded, obstacle;
        AdasMode current_mode;

        pthread_mutex_lock(&data_mutex);

        /* A reading older than STALE_TIMEOUT_NS is no longer trusted. */
        if (shared.ultra_last_good == 0 ||
            now - shared.ultra_last_good > STALE_TIMEOUT_NS)
            shared.ultrasonic_valid = 0;

        if (shared.ir_last_good == 0 ||
            now - shared.ir_last_good > STALE_TIMEOUT_NS)
            shared.ir_valid = 0;

        /*
         * Unchanged values are only a suspicion: a static scene may be valid.
         * These flags are logged, but do not alone force DEGRADED mode.
         */
        shared.ultra_stuck =
            shared.ultra_last_good != 0 &&
            shared.ultra_last_change != 0 &&
            now - shared.ultra_last_change > ULTRA_STUCK_NS;

        shared.ir_stuck =
            shared.ir_last_good != 0 &&
            shared.ir_last_change != 0 &&
            now - shared.ir_last_change > IR_STUCK_NS;

        shared.fusion_heartbeat = now;
        s = shared; /* Snapshot shared state, then process it outside the lock. */
        pthread_mutex_unlock(&data_mutex);

        ultra_fault = !s.ultrasonic_valid ||
                      s.ultrasonic_errors >= MAX_ULTRA_ERRORS ||
                      s.ultra_task_fault;
        ir_fault = !s.ir_valid || s.ir_task_fault;

        /* Compare sensors only when both readings are considered valid. */
        if (!ultra_fault && !ir_fault) {
            if (s.ir_obstacle && s.distance_cm > IR_FAR_CM)
                disagreement = 1;
            if (!s.ir_obstacle && s.distance_cm < IR_NEAR_CM)
                disagreement = 1;
        }

        /* Edge-triggered sensor fault/recovery messages. */
        if (ultra_fault != previous_ultra_fault) {
            if (ultra_fault) {
                printf("FAULT: ULTRASONIC SENSOR\n");
                log_event("SENSOR_FAULT: ULTRASONIC");
            } else {
                printf("RECOVERY: ULTRASONIC SENSOR\n");
                log_event("SENSOR_RECOVERED: ULTRASONIC");
            }
            previous_ultra_fault = ultra_fault;
        }

        if (ir_fault != previous_ir_fault) {
            if (ir_fault) {
                printf("FAULT: IR SENSOR\n");
                log_event("SENSOR_FAULT: IR");
            } else {
                printf("RECOVERY: IR SENSOR\n");
                log_event("SENSOR_RECOVERED: IR");
            }
            previous_ir_fault = ir_fault;
        }

        if (disagreement != previous_disagreement) {
            if (disagreement) {
                printf("FAULT: SENSOR DISAGREEMENT\n");
                log_event("SENSOR_DISAGREEMENT: DETECTED");
            } else {
                log_event("SENSOR_DISAGREEMENT: CLEARED");
            }
            previous_disagreement = disagreement;
        }

        if (s.ultra_stuck != previous_ultra_stuck) {
            log_event(s.ultra_stuck
                ? "SENSOR_SUSPECTED_STUCK: ULTRASONIC"
                : "SENSOR_STUCK_SUSPECT_CLEARED: ULTRASONIC");
            previous_ultra_stuck = s.ultra_stuck;
        }

        if (s.ir_stuck != previous_ir_stuck) {
            log_event(s.ir_stuck
                ? "SENSOR_SUSPECTED_STUCK: IR"
                : "SENSOR_STUCK_SUSPECT_CLEARED: IR");
            previous_ir_stuck = s.ir_stuck;
        }

        degraded = ultra_fault || ir_fault || disagreement;
        current_mode = degraded ? MODE_DEGRADED : MODE_NORMAL;
        set_mode(current_mode);

        if (current_mode == MODE_NORMAL) {
            obstacle = s.ir_obstacle || (s.distance_cm <= IR_NEAR_CM);

            /* Print on first result, mode recovery, or obstacle-state change. */
            if (first_result ||
                previous_mode != MODE_NORMAL ||
                obstacle != previous_obstacle) {
                printf("%s\n", obstacle
                    ? "OBSTACLE DETECTED" : "ROAD CLEAR");
                fflush(stdout);
                previous_obstacle = obstacle;
                first_result = 0;
            }
        } else {
            first_result = 1; /* Print a fresh result after recovery. */
        }

        previous_mode = current_mode;
        sleep_ms(FUSION_INTERVAL_MS);
    }

    return NULL;
}

/* Thread 4: monitor sensor/fusion heartbeat timestamps. */
static void *watchdog_thread(void *arg)
{
    int previous_ultra_stalled = 0;
    int previous_ir_stalled = 0;
    int previous_fusion_stalled = 0;
    (void)arg;

    for (;;) {
        uint64_t now = now_ns();
        int ultra_stalled, ir_stalled, fusion_stalled;

        pthread_mutex_lock(&data_mutex);
        ultra_stalled = shared.ultra_heartbeat == 0 ||
            now - shared.ultra_heartbeat > ULTRA_TASK_TIMEOUT_NS;
        ir_stalled = shared.ir_heartbeat == 0 ||
            now - shared.ir_heartbeat > IR_TASK_TIMEOUT_NS;
        fusion_stalled = shared.fusion_heartbeat == 0 ||
            now - shared.fusion_heartbeat > FUSION_TASK_TIMEOUT_NS;

        shared.ultra_task_fault = ultra_stalled;
        shared.ir_task_fault = ir_stalled;
        shared.fusion_task_fault = fusion_stalled;
        pthread_mutex_unlock(&data_mutex);

        if (ultra_stalled != previous_ultra_stalled) {
            log_event(ultra_stalled
                ? "TASK_STALLED: ULTRASONIC"
                : "TASK_RECOVERED: ULTRASONIC");
            if (ultra_stalled)
                printf("FAULT: ULTRASONIC TASK STALLED\n");
            previous_ultra_stalled = ultra_stalled;
        }

        if (ir_stalled != previous_ir_stalled) {
            log_event(ir_stalled
                ? "TASK_STALLED: IR"
                : "TASK_RECOVERED: IR");
            if (ir_stalled)
                printf("FAULT: IR TASK STALLED\n");
            previous_ir_stalled = ir_stalled;
        }

        if (fusion_stalled != previous_fusion_stalled) {
            log_event(fusion_stalled
                ? "TASK_STALLED: FUSION"
                : "TASK_RECOVERED: FUSION");
            if (fusion_stalled)
                printf("FAULT: FUSION TASK STALLED\n");
            previous_fusion_stalled = fusion_stalled;
        }

        /*
         * NOTE: fusion_thread must include fusion_task_fault in its mode
         * decision too. As originally structured, this watchdog can set
         * DEGRADED but the next fusion cycle can overwrite it. See notes
         * accompanying this file before treating this as safety behaviour.
         */
        if (ultra_stalled || ir_stalled || fusion_stalled)
            set_mode(MODE_DEGRADED);

        fflush(stdout);
        sleep_ms(WATCHDOG_INTERVAL_MS);
    }

    return NULL;
}

/* Thread 5: blink the warning LED in DEGRADED mode; otherwise keep it off. */
static void *led_thread(void *arg)
{
    int led_on = 0;
    (void)arg;

    for (;;) {
        AdasMode mode;

        pthread_mutex_lock(&data_mutex);
        mode = shared.mode;
        pthread_mutex_unlock(&data_mutex);

        if (mode == MODE_DEGRADED) {
            led_on = !led_on;
            gpio_write(LED_PIN, led_on);
            sleep_ms(250);
        } else {
            led_on = 0;
            gpio_write(LED_PIN, 0);
            sleep_ms(100);
        }
    }

    return NULL;
}

/* Main: initialize GPIO, initialize state, then create all worker threads. */
int main(void)
{
    pthread_t ultra_tid, ir_tid, fusion_tid;
    pthread_t watchdog_tid, logger_tid, led_tid;
    uint64_t start;

    printf("QNX ADAS Sensor Fusion Started\n");

    if (init_gpio() != 0)
        return EXIT_FAILURE;

    /* Seed heartbeats so startup is not immediately reported as a stall. */
    start = now_ns();
    pthread_mutex_lock(&data_mutex);
    shared.mode = MODE_NORMAL;
    shared.ultra_heartbeat = start;
    shared.ir_heartbeat = start;
    shared.fusion_heartbeat = start;
    pthread_mutex_unlock(&data_mutex);

    /* Start logger before the producers begin generating events. */
    if (pthread_create(&logger_tid, NULL, logger_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create logger thread\n");
        return EXIT_FAILURE;
    }
    if (pthread_create(&ultra_tid, NULL, ultrasonic_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create ultrasonic thread\n");
        return EXIT_FAILURE;
    }
    if (pthread_create(&ir_tid, NULL, ir_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create IR thread\n");
        return EXIT_FAILURE;
    }
    if (pthread_create(&fusion_tid, NULL, fusion_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create fusion thread\n");
        return EXIT_FAILURE;
    }
    if (pthread_create(&watchdog_tid, NULL, watchdog_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create watchdog thread\n");
        return EXIT_FAILURE;
    }
    if (pthread_create(&led_tid, NULL, led_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create LED thread\n");
        return EXIT_FAILURE;
    }

    /* Worker threads run forever; joins keep main alive. */
    (void)pthread_join(ultra_tid, NULL);
    (void)pthread_join(ir_tid, NULL);
    (void)pthread_join(fusion_tid, NULL);
    (void)pthread_join(watchdog_tid, NULL);
    (void)pthread_join(logger_tid, NULL);
    (void)pthread_join(led_tid, NULL);

    return EXIT_SUCCESS;
}
