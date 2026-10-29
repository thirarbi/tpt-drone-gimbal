#include "gimbal_state.h"
#include "esp_check.h"
#include "proto.h"

QueueHandle_t g_target_mb, g_qcmd_mb, g_qmeas_mb, g_ctrl_mb;

void mailboxes_init(void) {
    g_target_mb = xQueueCreate(1, sizeof(target_t));
    g_qcmd_mb   = xQueueCreate(1, sizeof(qcmd_t));
    g_qmeas_mb  = xQueueCreate(1, sizeof(qmeas_t));
    g_ctrl_mb   = xQueueCreate(1, sizeof(ctrl_out_t));
    ESP_ERROR_CHECK(g_target_mb && g_qcmd_mb && g_qmeas_mb && g_ctrl_mb ? ESP_OK : ESP_ERR_NO_MEM);

    const target_t boot = { GMODE_NADIR, -1.5707963f, 0.0f, 0.0f, false };   // §4.6 step 6
    xQueueOverwrite(g_target_mb, &boot);
}
