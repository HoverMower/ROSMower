/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// drivers (motor driver, sensor drivers, etc.)

#ifndef DRIVERS_H
#define DRIVERS_H

#include <Arduino.h>
#include <EEPROM.h>  


// returns sign of variable (-1, 0, +1)
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

// ---------- EEPROM helpers ----------------------------------

template <class T> int eewrite(int &ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int eeread(int &ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}

template <class T> int eereadwrite(boolean readflag, int &ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
    { 
       if (readflag) *p++ = EEPROM.read(ee++);
         else EEPROM.write(ee++, *p++);
    }
    return i;
}



#endif 
