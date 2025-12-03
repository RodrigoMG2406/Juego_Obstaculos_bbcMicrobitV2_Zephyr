/////////////////////////////////////////////////////////////////////////////
// Juego de “Esquiva los Obstáculos” para micro:bit v2 con Zephyr OS.
// - El jugador se mueve usando los botones A (izquierda) y B (derecha).
// - Se genera un obstáculo aleatorio que cae por la matriz LED 5x5.
// - Cada vez que el jugador esquiva un obstáculo suma 1 punto.
// - Cada 5 puntos, el juego acelera la caída del obstáculo.
// - El puntaje se envía por Bluetooth vía notificaciones GATT.
// - Al colisionar, se reproduce un sonido, se muestra GAME OVER
//   y el jugador vuelve automáticamente al centro.
//
// Plataforma: Zephyr OS + BBC micro:bit v2
/////////////////////////////////////////////////////////////////////////////

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>           // Control de botones
#include <zephyr/logging/log.h>            // Sistema de logs
#include <zephyr/display/mb_display.h>     // Display LED de micro:bit
#include <zephyr/drivers/pwm.h>            // Buzzer (PWM)
#include <zephyr/sys/printk.h>             // printk()
#include <zephyr/bluetooth/bluetooth.h>    // BLE core
#include <zephyr/bluetooth/gatt.h>         // Servicios y características BLE
#include <zephyr/bluetooth/hci.h>          // Controlador BLE
#include <zephyr/random/random.h>          // Random para obstáculos
#include <string.h>

LOG_MODULE_REGISTER(Juego, LOG_LEVEL_INF);

// ========================== Buzzer ===============================

#define SOUND_PERIOD_COLISION PWM_USEC(800)     // Frecuencia para colisión
#define SOUND_PERIOD_BTN      PWM_USEC(1500)    // Frecuencia para botones
#define SOUND_DURATION_MS     120               // Duración de beep en ms

// ======================= Configuración juego ======================

#define PLAYER_ROW 4       // Jugador siempre en la fila inferior
#define PLAYER_MIN 0       // Límite izquierdo
#define PLAYER_MAX 4       // Límite derecho

// Velocidad inicial del juego
int current_speed_ms = 500;

// Límite de aceleración
#define MIN_SPEED_MS 100
#define SPEED_STEP   50

int score = 0;             // Puntos del jugador

// ======================= Botones ================================

static int64_t a_timestamp;
static int64_t b_timestamp;

static const struct gpio_dt_spec sw0_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios); // Botón A
static const struct gpio_dt_spec sw1_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios); // Botón B

// Asegura que ambos botones están en el mismo controlador GPIO
BUILD_ASSERT(DT_SAME_NODE(DT_GPIO_CTLR(DT_ALIAS(sw0), gpios),
                           DT_GPIO_CTLR(DT_ALIAS(sw1), gpios)));

// ========================= PWM/Buzzer ============================

static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));

static inline void beep(uint32_t period){
    if (period == 0) {
        pwm_set_dt(&pwm, 0, 0);   // Apagar buzzer
        return;
    }
    pwm_set_dt(&pwm, period, period / 2); // Ciclo 50%
}

// Estados de sonido
static enum sound_state {
    SOUND_IDLE,
    SOUND_COLISION,
    SOUND_BTN,
} sound_state = SOUND_IDLE;

// Cambiar tono según el evento
static void sound_set(enum sound_state state){
    switch (state) {
    case SOUND_IDLE:      beep(0); break;
    case SOUND_COLISION:  beep(SOUND_PERIOD_COLISION); break;
    case SOUND_BTN:       beep(SOUND_PERIOD_BTN); break;
    }
    sound_state = state;
}

// ========================= BLE Advertising =========================

// Paquete de advertising: flags + UUID principal
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
        (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
        0xf0,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12,
        0xf0,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12)
};

// Paquete de scan response: nombre del dispositivo
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

// Parámetros de advertising BLE
static const struct bt_le_adv_param adv_params = {
    .options = BT_LE_ADV_OPT_CONN,                // Permite conexión
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
};

static struct bt_conn *current_conn = NULL;  // Conexión activa

// ===== CCC callback (cuando el cliente activa notificaciones) =====

static void score_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value){
    LOG_INF("Notificaciones del score %s",
           value == BT_GATT_CCC_NOTIFY ? "activadas" : "desactivadas");
}

// ====================== Servicio BLE ===============================

BT_GATT_SERVICE_DEFINE(button_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(
        0xf0,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12,
        0xf0,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12)),

    // Característica F1 — Notifica el score
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(
        0xf1,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12,
        0xf0,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12),
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, &score),

    BT_GATT_CCC(score_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

// ======================== Callbacks BLE ==============================

static void connected(struct bt_conn *conn, uint8_t err){
    if (err) {
        LOG_ERR("Falló conexión (err %u)", err);
        return;
    }
    current_conn = bt_conn_ref(conn); // Guardar conexión
    LOG_INF("Conectado.");
}

static void disconnected(struct bt_conn *conn, uint8_t reason){
    LOG_INF("Desconectado (razón %u)", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    // Reiniciar advertising
    int ret = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad),
                              sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Error al reiniciar advertising (%d)", ret);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

// Encender sistema BLE y arrancar advertising
static void configure_BT(void){
    if (bt_enable(NULL)) {
        LOG_ERR("Error inicializando BLE");
        return;
    }
    LOG_INF("Bluetooth listo.");

    if (bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad),
                        sd, ARRAY_SIZE(sd)))
        LOG_ERR("Error iniciando advertising");
}

// ================ Work para apagar sonido =====================

static struct k_work_delayable sound_off_work;

static void sound_off_worker(struct k_work *w){
    ARG_UNUSED(w);
    sound_set(SOUND_IDLE);
}

// ================ Lógica del juego ============================

// Posición inicial del jugador (centrado)
static volatile int player_x = 2;

// Estado del juego
static bool game_over = false;

// Imagen de cara triste para Game Over
static struct mb_image face[] = { MB_IMAGE(
    {0,1,0,1,0},
    {0,1,0,1,0},
    {0,0,0,0,0},
    {0,1,1,1,0},
    {1,0,0,0,1}
) };

// Trabajo periódico de refresco
static struct k_work_delayable refresh_work;

// Semáforo para actualizar display
static K_SEM_DEFINE(display_update, 0, 1);

// ================= Obstáculos =====================

uint8_t obstacle_x = 0;
uint8_t obstacle_y = 0;

// Genera frame actualizado del juego
void draw_frame(struct mb_image *img){
    uint8_t buf[5] = {0};

    // Dibujar jugador
    buf[4] |= (1 << (4 - player_x));

    // Dibujar obstáculo
    if (obstacle_y < 5)
        buf[obstacle_y] |= (1 << (4 - obstacle_x));

    memcpy(img->r, buf, 5);
}

// Genera nuevo obstáculo
void new_obstacle(void){
    obstacle_x = sys_rand32_get() % 5;
    obstacle_y = 0;
}

// =================== TICK DEL JUEGO =====================

static void game_tick(struct k_work *work){
    ARG_UNUSED(work);

    // Si hay colisión, solo refresca pantalla
    if (game_over){
        k_sem_give(&display_update);
        return;
    }

    // Avanza obstáculo
    obstacle_y++;

    // Si llegó al fondo → sumar punto
    if (obstacle_y > 4) {
        score++;

        // Enviar notificación BLE si hay cliente conectado
        if (current_conn)
            bt_gatt_notify(current_conn, &button_svc.attrs[2],
                           &score, sizeof(score));

        // Aumentar velocidad cada 5 puntos
        if (score % 5 == 0 && current_speed_ms > MIN_SPEED_MS)
            current_speed_ms -= SPEED_STEP;

        new_obstacle();
    }

    // Detección de colisión
    if (player_x == obstacle_x && obstacle_y == 4){
        game_over = true;
        k_sem_give(&display_update);
        return;
    }

    // Actualiza pantalla y programa próximo tick
    k_sem_give(&display_update);
    k_work_reschedule(&refresh_work, K_MSEC(current_speed_ms));
}

// ===================== Botones ============================

static struct gpio_callback button_cb_data;

// Callback de botón presionado
static void button_pressed(const struct device *dev,
                           struct gpio_callback *cb,
                           uint32_t pins){
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    bool a_pressed = pins & BIT(sw0_gpio.pin);
    bool b_pressed = pins & BIT(sw1_gpio.pin);

    // BOTÓN A → Mover izquierda
    if (a_pressed){
        if (k_uptime_delta(&a_timestamp) < 120) return;
        a_timestamp = k_uptime_get();

        if (player_x < PLAYER_MAX){
            player_x++;
            sound_set(SOUND_BTN);
            k_work_reschedule(&sound_off_work,
                              K_MSEC(SOUND_DURATION_MS));
            k_sem_give(&display_update);
        }
    }

    // BOTÓN B → Mover derecha
    if (b_pressed){
        if (k_uptime_delta(&b_timestamp) < 120) return;
        b_timestamp = k_uptime_get();

        if (player_x > PLAYER_MIN){
            player_x--;
            sound_set(SOUND_BTN);
            k_work_reschedule(&sound_off_work,
                              K_MSEC(SOUND_DURATION_MS));
            k_sem_give(&display_update);
        }
    }
}

// Configuración inicial de botones
static void configure_buttons(void){
    gpio_pin_configure_dt(&sw0_gpio, GPIO_INPUT);
    gpio_pin_configure_dt(&sw1_gpio, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&sw0_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&sw1_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_cb_data, button_pressed,
                       BIT(sw0_gpio.pin) | BIT(sw1_gpio.pin));

    gpio_add_callback(sw0_gpio.port, &button_cb_data);
}

// =========================== MAIN ================================

int main(void){
    struct mb_display *disp = mb_display_get();
    struct mb_image frame;

    // Inicialización general
    configure_buttons();
    configure_BT();

    if (!pwm_is_ready_dt(&pwm)){
        printk("PWM no disponible\n");
        return 0;
    }

    k_work_init_delayable(&refresh_work, game_tick);
    k_work_init_delayable(&sound_off_work, sound_off_worker);

    // Inicia lógica del juego
    k_work_schedule(&refresh_work, K_MSEC(current_speed_ms));

    // Pantalla inicial
    sound_set(SOUND_IDLE);
    mb_display_print(disp, MB_DISPLAY_MODE_DEFAULT,
                     1 * MSEC_PER_SEC, "ESQUIVA");
    k_sleep(K_MSEC(7000));

    // Estado inicial del juego
    player_x = 2;      // jugador centrado
    score = 0;
    current_speed_ms = 500;
    new_obstacle();

    // Loop principal
    while (1){
        k_sem_take(&display_update, K_FOREVER);

        if (game_over){
            // Animación de Game Over
            sound_set(SOUND_COLISION);
            k_work_reschedule(&sound_off_work,
                              K_MSEC(SOUND_DURATION_MS));

            mb_display_image(disp, MB_DISPLAY_MODE_SINGLE,
                             3000, face, 1);

            k_msleep(500);
            mb_display_print(disp, MB_DISPLAY_MODE_DEFAULT,
                             1000, "GAMEOVER");
            k_sleep(K_MSEC(8000));

            // Reinicio completo del juego
            score = 0;
            game_over = false;
            player_x = 2;             // Recolocar al centro
            current_speed_ms = 500;
            new_obstacle();

            // Reanudar
            k_sem_give(&display_update);
            k_work_reschedule(&refresh_work,
                              K_MSEC(current_speed_ms));
            continue;
        }

        // Mostrar frame actualizado
        draw_frame(&frame);
        mb_display_image(disp, MB_DISPLAY_MODE_SINGLE,
                         SYS_FOREVER_MS, &frame, 1);

        // Evita saturar CPU
        k_msleep(50);
    }

    return 0;
}

