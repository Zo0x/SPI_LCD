//
// LCD display test program
//
// Copyright (c) 2017 Larry Bank
// email: bitbank@pobox.com
// Project started 5/17/2017
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include "main.h"


int main()
{
    std::cout << "Loading...\n";

    int rc;
    int width=128, height=128;

    std::unique_ptr<SpiLcd> display(new SpiLcd);

    // Initialize the library on SPI channel 0
    // The pin numbers are for 40-pin headers on RPi2, RPi3, RPi0
    // Pass it the GPIO pin numbers used for the following:
    rc = display->spilcdInit(LCD_SSD1351, 0, 0, 1600000, 18, 22, -1); // LCD type, flip 180, SPI Channel, Freq, D/C, RST, LED
    if (rc != 0)
    {
        printf("Problem initializing spilcd library\n");
        return 0;
    }
    std::cout << "SPI library loaded\n";

    display->Demo(width, height);

    // Quit library and free resources
    display->spilcdShutdown();

    return 0;
}