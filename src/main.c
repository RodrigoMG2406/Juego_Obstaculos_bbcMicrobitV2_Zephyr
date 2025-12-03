#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>           // Control de pines GPIO
#include <zephyr/logging/log.h>            // Sistema de logs
#include <zephyr/display/mb_display.h>     // Manejo de display de micro:bit
#include <zephyr/drivers/pwm.h>            //Manejo del buzzer
#include <zephyr/sys/printk.h>             //comando print
#include <zephyr/bluetooth/bluetooth.h>    // Funciones generales BLE
#include <zephyr/bluetooth/gatt.h>         // Manejo de servicios y características GATT
#include <zephyr/bluetooth/hci.h>          // Interacción con el controlador BLE bajo nivel
#include <zephyr/random/random.h>
#include <string.h>

LOG_MODULE_REGISTER(Juego, LOG_LEVEL_INF);

#define SOUND_PERIOD_COLISION PWM_USEC(800)
#define SOUND_PERIOD_BTN    PWM_USEC(1500)
#define SOUND_DURATION_MS    120   // cuánto dura el sonido en ms 


// ===== CONFIGURACIÓN GENERAL ===== 

#define PLAYER_ROW     4      // Fila inferior 
#define PLAYER_MIN     0
#define PLAYER_MAX     4

// ===== Velocidad dinámica del juego =====
int current_speed_ms = 500;       // velocidad inicial
#define MIN_SPEED_MS 100          // velocidad mínima
#define SPEED_STEP   50          // cuanto se acelera cada 5 puntos
int score = 0;

// ===== CONFIGURACIÓN BOTONES ===== 
static int64_t a_timestamp;
static int64_t b_timestamp;

static const struct gpio_dt_spec sw0_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec sw1_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

// ensure SW0 & SW1 are on same gpio controller 
BUILD_ASSERT(DT_SAME_NODE(DT_GPIO_CTLR(DT_ALIAS(sw0), gpios), DT_GPIO_CTLR(DT_ALIAS(sw1), gpios)));

// ===== CONFIGURACIÓN BUZZER =====
static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));

static inline void beep(uint32_t period){
    if (period == 0) {
        pwm_set_dt(&pwm, 0, 0);
        return;
    }
    pwm_set_dt(&pwm, period, period / 2);
}

static enum sound_state {
    SOUND_IDLE,
    SOUND_COLISION,
    SOUND_BTN,
} sound_state = SOUND_IDLE;

static void sound_set(enum sound_state state){
    switch (state) {
    case SOUND_IDLE: beep(0); break;
    case SOUND_COLISION: beep(SOUND_PERIOD_COLISION); break;
    case SOUND_BTN: beep(SOUND_PERIOD_BTN); break;
    }
    sound_state = state;
}

// PUBLICIDAD BLE (ADVERTISING)
// Paquete de advertising: contiene flags + UUID del servicio principal (128 bits)
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    // UUID de 128 bits (espejeado por protocolo)
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12)
};

// Paquete de scan response: muestra el nombre del dispositivo
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

// Parámetros de advertising BLE
static const struct bt_le_adv_param adv_params = {
    .options = BT_LE_ADV_OPT_CONN,         // Permite conexiones
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2, // Intervalos rápidos
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
};

static struct bt_conn *current_conn;  // Guardará la conexión BLE activa

// ==========================================================
//  CCC — CLIENT CHARACTERISTIC CONFIGURATION
//  Se llama cuando un cliente activa/desactiva notificaciones
// ==========================================================

static void score_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value){
    LOG_INF("Notificaciones score %s", score == BT_GATT_CCC_NOTIFY ? "habilitadas" : "deshabilitadas");
}

BT_GATT_SERVICE_DEFINE(button_svc,
    // Servicio principal UUID F0
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(
        0xf0,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12,
        0xf0,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12)),

    // ----------- CARACTERÍSTICA BOTÓN A (UUID F1) -----------
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(
        0xf1,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12,
        0xf0,0xde,0xbc,0x9a,0x78,0x56,0x34,0x12),
        BT_GATT_CHRC_NOTIFY,     // Solo notificación
        BT_GATT_PERM_NONE,       // Sin permisos de lectura/escritura
        NULL, NULL, &score),     // El valor a enviar es "score"
    BT_GATT_CCC(score_ccc_cfg_changed,BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
    );
    
// ==========================================================
// CALLBACKS DE CONEXIÓN BLE
// ==========================================================

static void connected(struct bt_conn *conn, uint8_t err){
    if (err) {
        LOG_ERR("Hay un fallo en la conexion :( (err %u)", err);
    } else {
        current_conn = bt_conn_ref(conn);  // Guarda la conexión
        LOG_INF("Conectado :) ");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason){
    LOG_INF("Se desconectó por... (razon # %u)", reason);

    // Borra la referencia a la conexión
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    // Reinicia advertising
    int ret = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("No se puedo reiniciar el advertising (err %d)", ret);
    } else {
        LOG_INF("Se logra sin problema el advertising");
    }
}

// Registra los callbacks
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void configure_BT(void){
    int ret;
    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("No funciono el BLE :( (err %d)", ret);
        return 0;
    }
    LOG_INF("Bluetooth OK");

    // Inicia advertising
    ret = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("El Advertising no fue posible :( (err %d)", ret);
        return 0;
    }
    LOG_INF("El Advertising funciona, busca a: %s", CONFIG_BT_DEVICE_NAME);

}

// trabajo delayable para apagar sonido //
static struct k_work_delayable sound_off_work;
static void sound_off_worker(struct k_work *w) { ARG_UNUSED(w); sound_set(SOUND_IDLE); }

// ===== LOGICA DEL JUEGO ===== //

static volatile int player_x = 2;   //jugador inicia en medio 


static bool game_over = false;
static struct mb_image face[] = {MB_IMAGE({0,1,0,1,0},
                                           {0,1,0,1,0},
                                           {0,0,0,0,0},
                                           {0,1,1,1,0},
                                           {1,0,0,0,1})};
                                           
// trabajo periódico
static struct k_work_delayable refresh_work;
static K_SEM_DEFINE(display_update, 0, 1);
                                           

// ===== GENERACIÓN DE OBSTÁCULOS ===== //

uint8_t obstacle_y = 0;
uint8_t obstacle_x = 0;

void draw_frame(struct mb_image *img)
{
    uint8_t buf[5] = {0};

    // Jugador
    buf[4] |= (1 << (4 - player_x));

    // Obstáculo
    if (obstacle_y < 5) {
        buf[obstacle_y] |= (1 << (4 - obstacle_x));

    }

    memcpy(img->r, buf, 5);
}


void new_obstacle(void){
    obstacle_x = sys_rand32_get() % 5;
    obstacle_y = 0;
}


// ===== ACTUALIZACIÓN DEL JUEGO ===== //

static void game_tick(struct k_work *work){
    ARG_UNUSED(work);

    if (game_over) {
        k_sem_give(&display_update);
        return;
    }
    
    //Mover Obsrtaculo
    obstacle_y++;

    if (obstacle_y > 4) {
        score++;
        if (current_conn) {
            // attrs[2] contiene el valor de la característica F1
            bt_gatt_notify(current_conn, &button_svc.attrs[2], &score, sizeof(score));
        } else {
            LOG_INF("No hay cliente que reciba score");
        }
        //ACELERACIÓN CADA 5 PUNTOS
        if (score > 0 && score % 5 == 0) {
            if (current_speed_ms > MIN_SPEED_MS) {
                current_speed_ms -= SPEED_STEP;
            }
        }
        new_obstacle();
    }
    
    //Colision
    if (player_x == obstacle_x && obstacle_y == 4) {
        game_over = true;
        k_sem_give(&display_update);
        return;
    }

    k_sem_give(&display_update);
    k_work_reschedule(&refresh_work, K_MSEC(current_speed_ms));
}


/* ===== MANEJO DE BOTONES ===== */
static struct gpio_callback button_cb_data;
static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    bool a_pressed = pins & BIT(sw0_gpio.pin); /* sw0 = A */
    bool b_pressed = pins & BIT(sw1_gpio.pin); /* sw1 = B */

    /* Debounce y actualizar timestamp */
    if (a_pressed) {
        if (k_uptime_delta(&a_timestamp) < 120) {
            return;
        }
        a_timestamp = k_uptime_get();
        /* A — mover izquierda */
        if (player_x < PLAYER_MAX) {
            player_x++;
            sound_set(SOUND_BTN);
            k_work_reschedule(&sound_off_work, K_MSEC(SOUND_DURATION_MS));
            k_sem_give(&display_update);
        }
    }

    if (b_pressed) {
        if (k_uptime_delta(&b_timestamp) < 120) {
            return;
        }
        b_timestamp = k_uptime_get();
        /* B — mover derecha */
        if (player_x > PLAYER_MIN) {
            player_x--;
            sound_set(SOUND_BTN);
            k_work_reschedule(&sound_off_work, K_MSEC(SOUND_DURATION_MS));
            k_sem_give(&display_update);
        }
    }
}

static void configure_buttons(void){
    if (!gpio_is_ready_dt(&sw0_gpio)) {
        printk("%s: device not ready.\n", sw0_gpio.port->name);
        return;
    }

    gpio_pin_configure_dt(&sw0_gpio, GPIO_INPUT);
    gpio_pin_configure_dt(&sw1_gpio, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&sw0_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&sw1_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_cb_data, button_pressed,
                       BIT(sw0_gpio.pin) | BIT(sw1_gpio.pin));

    gpio_add_callback(sw0_gpio.port, &button_cb_data);
}


  
/////////////////////////////////////////////////////////////////////////////////////////
int main(void){
    struct mb_display *disp = mb_display_get();
    struct mb_image frame;

    configure_buttons(); //Inicaializa Botones
    configure_BT(); //Inicializa bluetooth
    
    if (!pwm_is_ready_dt(&pwm)) {
		printk("%s: device not ready.\n", pwm.dev->name);
		return 0;
	}
    
    //inicializa los work
    k_work_init_delayable(&refresh_work, game_tick);
    k_work_init_delayable(&sound_off_work, sound_off_worker);
    
    //Inicia tasa de refresco del juego
    k_work_schedule(&refresh_work, K_MSEC(current_speed_ms));
    
    //Pantalla Inicial del juego
    sound_set(SOUND_IDLE);
    mb_display_print(disp, MB_DISPLAY_MODE_DEFAULT, 1*MSEC_PER_SEC, "ESQUIVA");
    k_sleep(K_MSEC(7000));
    
    // iniciar primer obstáculo
    player_x = 2;
    score = 0;
    current_speed_ms = 500;   // velocidad inicial
    new_obstacle();

    while (1) {
        k_sem_take(&display_update, K_FOREVER);

        if (game_over) {
            //mostrar cara y mensaje, reiniciar al centro y reanudar juego
            sound_set(SOUND_COLISION);
            k_work_reschedule(&sound_off_work, K_MSEC(SOUND_DURATION_MS));
            mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, 3000, face, 1);
            k_msleep(500);
            mb_display_print(disp, MB_DISPLAY_MODE_DEFAULT, 1*MSEC_PER_SEC, "GAMEOVER");
            k_sleep(K_MSEC(8000));

            /* reinicio */
            score = 0;
            game_over = false;
            player_x = 2; //centra al personaje
            current_speed_ms = 500;     // reinicia velocidad
            new_obstacle();
            k_sem_give(&display_update);
            k_work_reschedule(&refresh_work, K_MSEC(current_speed_ms));
            continue;
        }

        draw_frame(&frame);
        mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, SYS_FOREVER_MS, &frame, 1);

        // pequeña latencia para no saturar CPU en loop principal
        k_msleep(50);
    }

    return 0;
}
