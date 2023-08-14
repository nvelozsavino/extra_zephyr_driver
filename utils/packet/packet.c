
#include <string.h>
#include "packet.h"
#include "../utils.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(packet, LOG_LEVEL_INF);



#define PACKET_THREAD_STACK_SIZE 3000

K_THREAD_STACK_DEFINE(packet_thread_stack, PACKET_THREAD_STACK_SIZE);
static struct k_thread packet_thread_data;

typedef struct {
	bool initiated;
	uint8_t state;
	int64_t last;
	packet_t packet;
	uint32_t data_ptr;
	uint16_t crc;
	uint8_t activeBuffer;
	struct k_sem tx_sem;
	const struct device* dev;
	packet_process_t* callback;
} packet_handler_t;

static packet_handler_t m_packet = {
	.initiated=false
};

K_MSGQ_DEFINE(rx_msgq, sizeof(packet_t), 10, 4);


// CRC Table
static const uint16_t crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
		0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
		0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
		0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
		0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
		0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
		0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
		0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
		0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
		0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
		0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
		0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
		0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
		0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
		0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
		0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
		0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
		0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
		0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
		0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
		0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
		0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
		0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
		0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
		0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

static uint16_t crc16(const uint8_t *buf, uint32_t len) {
	uint32_t i;
	uint16_t cksum = 0;
	for (i = 0; i < len; i++) {
		cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
	}
	return cksum;
}


static void serial_rx_callback(const struct device *dev, void *user_data)
{
    uint8_t data=0;
    if (!uart_irq_update(dev)) {
        return;
    }
	
	int64_t now = k_uptime_get();

	if (m_packet.state!=0 && (k_uptime_get()-m_packet.last)>CONFIG_PACKET_RX_TIMEOUT){
		LOG_DBG("PTO");
		m_packet.state=0;
	}
    while (uart_irq_rx_ready(dev)) {

        uart_fifo_read(dev, &data, 1);

		switch (m_packet.state) {
		case 0:
			if (data == 0x02) {
				// 1 byte PL len
				//TM_LCD_Puts("Packet ");
				m_packet.state += 2;
				m_packet.last = now;
				m_packet.data_ptr = 0;
				m_packet.packet.length = 0;
				//i=4;
			} else if (data == 0x03) {
				// 2 byte PL len
				m_packet.state++;
				m_packet.last = now;
				m_packet.data_ptr = 0;
				m_packet.packet.length = 0;
			} else {
				LOG_DBG("PSB");
				m_packet.state = 0;
			}
			break;

		case 1:
			m_packet.packet.length = (uint16_t)data << 8;
			m_packet.state++;
			m_packet.last = now;
			break;

		case 2:
			m_packet.packet.length |= (uint16_t)data;
			if (m_packet.packet.length > 0 && m_packet.packet.length <= CONFIG_PACKET_MAX_PL_LEN) {
				m_packet.state++;
				m_packet.last = now;
			} else {
				LOG_DBG("PPL");
				m_packet.state = 0;
			}
			break;

		case 3:
			m_packet.packet.data[m_packet.data_ptr++] = data;
			if (m_packet.data_ptr == m_packet.packet.length) {
				m_packet.state++;
			}
			m_packet.last = now;
			break;

		case 4:
			m_packet.crc = (uint16_t)data<<8;
			m_packet.state++;
			m_packet.last = now;
			break;

		case 5:
			m_packet.crc|= (uint16_t)data;
			m_packet.state++;
			m_packet.last = now;
			break;

		case 6:
			if (data == 0x03) {
				if (crc16(m_packet.packet.data, m_packet.packet.length) == m_packet.crc) {

					k_msgq_put(&rx_msgq,&m_packet.packet, K_NO_WAIT);
					// Packet received!
					// if (m_packet.process_func) {
                    //     m_packet.process_func(m_packet.custom_data, m_packet..,
					// 			m_packet.payload_length);
					// }else{
					// 	LOG_DEBUG("PNF%d\r\n",(int)handler_num);
					// }
				}else{
					LOG_DBG("PCRC");
				}
			}else{
				LOG_DBG("PEB");
			}
			m_packet.state = 0;
			break;

		default:
			LOG_DBG("PUS");
			m_packet.state = 0;
			break;
		}

    }
}


static void packet_thread(void *arg1, void *arg2, void *arg3)
{
    packet_t packet;

    uart_irq_callback_user_data_set(m_packet.dev, serial_rx_callback, NULL);
    uart_irq_rx_enable(m_packet.dev);

    while (k_msgq_get(&rx_msgq, &packet, K_FOREVER) == 0) {

        packet_process_t* callback=m_packet.callback;
        while (callback!=NULL){
            if (callback->func!=NULL){
                if (callback->func(packet.data,packet.length,callback->context)){
					break;
				}
            }
            callback=callback->next;
        }
    }
}

int packet_send(const uint8_t *data, uint16_t len, k_timeout_t timeout)
{
	if (!m_packet.initiated){
		return -EACCES;
	}
    int res = k_sem_take(&m_packet.tx_sem, timeout);
	if (res!=0){
		return res;
	}
	if (len>255){
		uart_poll_out(m_packet.dev, 0x02);
		uart_poll_out(m_packet.dev, (uint8_t)len);
	} else {
		uart_poll_out(m_packet.dev, 0x03);
		uart_poll_out(m_packet.dev, (uint8_t)(len>>8));
		uart_poll_out(m_packet.dev, (uint8_t)(len & 0xFF));
	}
    for (int i = 0; i < len; i++) {
        uart_poll_out(m_packet.dev, data[i]);
    }
	uint16_t crc = crc16(data,len);
	uart_poll_out(m_packet.dev, (uint8_t)(crc>>8));
	uart_poll_out(m_packet.dev, (uint8_t)(crc & 0xFF));
	uart_poll_out(m_packet.dev, 0x03);

    k_sem_give(&m_packet.tx_sem);
	return 0;
}


int packet_init(const struct device* serial)
{
	if (m_packet.initiated){
		LOG_INF("Packet already initiated");
		return 0;
	}
	m_packet.dev=serial;
    k_sem_init(&m_packet.tx_sem, 1, 1);
    if (!device_is_ready(serial)) {
        LOG_ERR("Serial device not ready");
        return -ENODEV;
    }
	m_packet.initiated=true;
	m_packet.callback=NULL;
    k_thread_create(&packet_thread_data, packet_thread_stack,
                    K_THREAD_STACK_SIZEOF(packet_thread_stack), packet_thread, NULL, NULL,
                    NULL, CONFIG_PACKET_THREAD_PRIORITY, 0, K_NO_WAIT);

	
    return 0;
}



int packet_register(packet_process_t* cb){
	if (!m_packet.initiated){
		return -EACCES;
	}
    if (cb==NULL){
        return -EINVAL;
    }
    cb->next=m_packet.callback;
    m_packet.callback=cb;
    return 0;
}

int packet_unregister(packet_process_t* cb){
	if (!m_packet.initiated){
		return -EACCES;
	}
    if (cb==NULL){
        return -EINVAL;
    }
    packet_process_t** next = &m_packet.callback;
    while (*next!=NULL){
        if (*next==cb){
            *next=cb->next;
            cb->next=NULL;
            return 0;
        }
        next=&cb->next;
    }
    return -ENOENT;
}

