/*
   MERG CBUS Module Library - RasberryPi Pico SDK port
   Copyright (c) Kevin Kimber 2023

   Based on work by Duncan Greenwood
   Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

   This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

#include "CBUSWiFi.h"

#include <pico/stdlib.h>
#include <pico/cyw43_arch.h>
#include "lwip/apps/httpd.h"

///-------------------
// TEST CODE

// SSI tags - tag length limited to 8 bytes by default
const char *ssi_tags[] = {"ipaddr", "port", "mac"};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen)
{
   size_t printed;
   switch (iIndex)
   {
   case 0: // ipaddr
   {
      printed = sprintf(pcInsert, "%s", ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[0])));

      //ip4_addr_t ipAddr = netif_ip4_addr(netif);
      //printed = snprintf(pcInsert, iInsertLen, "%d.%d.%d.%d",
      //   static_cast<uint8_t>(ipAddr.addr & 0xFF),
      //   static_cast<uint8_t>((ipAddr.addr >> 8) & 0xFF),
      //   static_cast<uint8_t>((ipAddr.addr >> 16) & 0xFF),
      //   static_cast<uint8_t>((ipAddr.addr >> 24)) & 0xFF);
   }
   break;
   case 1: // port
   {
      printed = snprintf(pcInsert, iInsertLen, "%d", 55555);
   }
   break;
   case 2: // mac
   {
      printed = snprintf(pcInsert, iInsertLen, "%x:%x:%x:%x:%x:%x", 6, 5, 4, 3, 2, 1);
   }
   break;
   default:
      printed = 0;
      break;
   }

   return (u16_t)printed;
}

// Initialise the SSI handler
void ssi_init(void)
{
   http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}

///-------------------

///
/// A class to encapsulate a physical pushbutton switch, with non-blocking processing
///

CBUSWiFi::CBUSWiFi()
{
}

///
/// @brief Initialize client connection to WiFi router
///
/// @return true Successfully connected
/// @return false Failed to connect to the WiFi router
bool CBUSWiFi::InitializeClient()
{
   char ssid[] = "XXXXX"; // TODO NEVER COMMIT WITH REAL CREDENTIALS
   char pass[] = "XXXXX";

   if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK))
   {
      return false;
   }

   cyw43_arch_enable_sta_mode();

   if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 10000))
   {
      return false;
   }

   return true;
}

///
/// @brief Initialize WiFi as aa local hotspot access point
///
/// @return true
/// @return false
///
bool CBUSWiFi::InitializeAP()
{
   return true;
}

bool CBUSWiFi::InitWebServer()
{
   // Initialise web server
   httpd_init();

   // Configure SSI and CGI handler
   ssi_init();

   return true;
}
