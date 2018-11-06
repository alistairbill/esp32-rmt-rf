#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "Arduino.h"

#include "esp32-hal.h"

rmt_obj_t* rmt_recv = NULL;

inline bool rf_valid(rmt_data_t* i)
{
    return (i->level0 && !i->level1 && i->duration0 >= 25 && i->duration1 >= 25);
}

inline bool in_range(int duration_ticks, int target_us, int margin_us)
{
    return (duration_ticks < (target_us + margin_us)) 
        && (duration_ticks > (target_us - margin_us));
}

inline bool bit_zero_if(rmt_data_t* item)
{
    return (in_range(item->duration0, 35, 10) && in_range(item->duration1, 110, 10));
}

inline bool bit_one_if(rmt_data_t* item)
{
    return (in_range(item->duration0, 110, 10) && in_range(item->duration1, 35, 10));
}

void receive_data(uint32_t *data, size_t len)
{
    if (len < 25) return;
    rmt_data_t* items = (rmt_data_t*)data;
    uint32_t buf = 0;
    for (size_t i = 0; i < len - 1; i++) {
        if (!rf_valid(&items[i])) return;
        if (bit_one_if(&items[i])) {
            // one
            buf |= (1 << (len - 1 - i));
        } else if (bit_zero_if(&items[i])) {
            // zero
        } else return;
    }
    if (items[len - 1].level0 && in_range(items[len - 1].duration0, 35, 10)) buf |= (1 << (len - 1));
    Serial.printf("Received: %d items: addr: %d cmd: %d\n", len, buf & 0xfffff, buf & 0xf);
}

void setup() 
{
    Serial.begin(115200);
    
    // Initialize the channel to capture up to 192 items
    if ((rmt_recv = rmtInit(15, false, RMT_MEM_192)) == NULL)
    {
        Serial.println("init receiver failed\n");
    }

    // Setup 10us tick
    float realTick = rmtSetTick(rmt_recv, 10000);
    Serial.printf("real tick set to: %fns\n", realTick);

    rmtSetFilter(rmt_recv, true, 200);

    // Ask to start reading
    rmtRead(rmt_recv, receive_data);
}

void loop() 
{
}
