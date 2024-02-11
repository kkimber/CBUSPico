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

// FatFs support
#include "f_util.h"
#include "ff.h"
#include "hw_config.h"

/// Helper for INI parsing
#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0

/// @brief Parser for config.ini
static int handler(void *user, const char *section, const char *name, const char *value)
{
   config_t *pconfig = (config_t *)user;

   // Configuration INI section headers
   constexpr const char *wifiSection = "wifi";
   constexpr const char *gcSection = "gridconnect";
   constexpr const char *edSection = "edthrottle";

   if (MATCH(wifiSection, "country"))
   {
      pconfig->country = strdup(value);
   }
   else if (MATCH(wifiSection, "ssid"))
   {
      pconfig->ssid = strdup(value);
   }
   else if (MATCH(wifiSection, "password"))
   {
      pconfig->passwd = strdup(value);
   }
   else if (MATCH(wifiSection, "wpa_auth"))
   {
      pconfig->wpaAuth = false; // todo
   }
   else if (MATCH(wifiSection, "wpa2_auth"))
   {
      pconfig->wpa2Auth = true; // todo
   }
   else if (MATCH(gcSection, "enable"))
   {
      pconfig->gcEnable = strcasecmp(value, "true") == 0 ? true : false;
   }
   else if (MATCH(gcSection, "port"))
   {
      pconfig->gcPort = atoi(value);
   }
   else if (MATCH(edSection, "enable"))
   {
      pconfig->edEnable = strcasecmp(value, "true") == 0 ? true : false;
   }
   else if (MATCH(edSection, "port"))
   {
      pconfig->edPort = atoi(value);
   }
   else
   {
      return 0; /* unknown section/name, error */
   }

   return 1;
}

////////////////////////////////
///// TEST WEB SERVER CODE

// SSI tags - tag length limited to 8 bytes by default
const char *ssi_tags[] = {"gcenable","gcport","edenable","edport"};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen)
{
   size_t printed;
   switch (iIndex)
   {
   case 0: // Grid Connect server enabled
   {
      if (CBUSWiFi::isGridConnectEnabled())
      {
         printed = snprintf(pcInsert, iInsertLen, "%s", "Yes");
      }  
      else
      {
         printed = snprintf(pcInsert, iInsertLen, "%s", "No");
      }
      break;
   }
   case 1: // Grid Connect port
   {
      printed = snprintf(pcInsert, iInsertLen, "%d", CBUSWiFi::getGridConnectPort());
   }
   break;
   case 2: // Engine Driver Throttle enabled
   {
      if (CBUSWiFi::isEdThrottleEnabled())
      {
         printed = snprintf(pcInsert, iInsertLen, "%s", "Yes");
      }  
      else
      {
         printed = snprintf(pcInsert, iInsertLen, "%s", "No");
      }
      break;
   }
   case 3: // Engine Driver Throttle port
   {
      printed = snprintf(pcInsert, iInsertLen, "%d", CBUSWiFi::getEdThrottlePort());
   }
   break;
   default:
      printed = 0;
      break;
   }

   return static_cast<u16_t>(printed);
}

// Initialise the SSI handler
void ssi_init(void)
{
   http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}

///// TEST WEB SERVER CODE
////////////////////////////////


///
/// A class to manage WiFi to CBUS connectivity
///
CBUSWiFi::CBUSWiFi()
{
}

///
/// @brief Reads a configuration file "config.ini" on the SD card
/// 
/// @return true if the SD card could be mounted and the configuration file parsed
/// @return false the SD card could not be mounted, or error parsing the configuration file
///
bool CBUSWiFi::ReadConfiguration()
{
   // Setup SPI based on configuration in hw_config.c
   sd_card_t *pSD = sd_get_by_num(0);

   // Mount the SD card as a FAT filesystem
   if (f_mount(&pSD->fatfs, pSD->pcName, 1) != FR_OK)
   {
      // Could not mount the file system
      return false;
   }

   // Parse the configuration file into our configuration
   int result = parseINI("config.ini", handler, &m_config);

   // Unmount SD card
   f_unmount(pSD->pcName);

   return result == 0;
}

///
/// @brief Initialize client connection to WiFi router
///
/// @return true Successfully connected
/// @return false Failed to connect to the WiFi router
bool CBUSWiFi::InitializeClient()
{
   // Read the configuration INI
   if (!ReadConfiguration())
   {
      // Could not setup SPI, mount the SD card or parse the configuration
      return false;
   }

   // Set the country code 
   /// @todo setup from INI
   if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK))
   {
      return false;
   }

   cyw43_arch_enable_sta_mode();

   // Attempt connection to the WiFi router
   if (cyw43_arch_wifi_connect_timeout_ms(m_config.ssid, m_config.passwd, CYW43_AUTH_WPA2_AES_PSK, 10000))
   {
      // Failed to connect to WiFi
      return false;
   }

   // WiFi connected
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
   /// @todo AP setup
   return true;
}

///
/// @brief Initialize a web server to manage configuration
///
/// @return true if the web server was successfully initiaized
/// @return false if the web server was not initialized
///
bool CBUSWiFi::InitWebServer(void)
{
   // Initialise web server
   httpd_init();

   // Configure SSI and CGI handler
   ssi_init();

   return true;
}

int CBUSWiFi::parseINI(const char *filename, ini_handler handler, void *user)
{
   FRESULT result;
   FIL file;
   int error;

   result = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);

   if (result != FR_OK)
   {
      return -1;
   }

   error = ini_parse_stream((ini_reader)f_gets, &file, handler, user);

   f_close(&file);

   return error;
}
