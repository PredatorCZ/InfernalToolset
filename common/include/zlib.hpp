/*  Infernal zlib shared methods
    Copyright(C) 2022 Lukas Cone

    This program is free software : you can redistribute it and / or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once
#include "settings.hpp"
#include <string>

void IT_EXTERN ProcessZAPStream(std::string &inBuffer, std::string &outBuffer,
                                size_t availIn);
size_t IT_EXTERN ProcessVFSStream(std::string &inBuffer, std::string &outBuffer,
                                  size_t availIn);
