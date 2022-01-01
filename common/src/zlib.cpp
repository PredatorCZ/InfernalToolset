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

#include "zlib.hpp"
#include "zlib.h"
#include <cstring>
#include <stdexcept>

void ProcessZAPStream(std::string &inBuffer, std::string &outBuffer,
                      size_t availIn) {
  z_stream infstream;
  infstream.zalloc = Z_NULL;
  infstream.zfree = Z_NULL;
  infstream.opaque = Z_NULL;
  infstream.avail_in = availIn;
  infstream.next_in = reinterpret_cast<Bytef *>(&inBuffer[0]);
  infstream.avail_out = outBuffer.size();
  infstream.next_out = reinterpret_cast<Bytef *>(&outBuffer[0]);
  inflateInit2(&infstream, -MAX_WBITS);
  int state = inflate(&infstream, Z_FINISH);
  inflateEnd(&infstream);

  if (state < 0) {
    if (infstream.msg) {
      throw std::runtime_error(infstream.msg);
    } else {
      throw std::runtime_error("zlib error: " + std::to_string(state));
    }
  }
}

size_t ProcessVFSStream(std::string &inBuffer, std::string &outBuffer,
                        size_t availIn) {
  z_stream infstream;
  infstream.zalloc = Z_NULL;
  infstream.zfree = Z_NULL;
  infstream.opaque = Z_NULL;
  infstream.avail_in = availIn;
  infstream.next_in = reinterpret_cast<Bytef *>(&inBuffer[0]);
  infstream.avail_out = outBuffer.size();
  infstream.next_out = reinterpret_cast<Bytef *>(outBuffer.data());
  inflateInit(&infstream);
  int state = inflate(&infstream, Z_FINISH);
  inflateEnd(&infstream);

  if (state < 0) {
    if (infstream.msg) {
      // use other flush type?
      if (!strcmp("incorrect data check", infstream.msg)) {
        throw std::runtime_error(infstream.msg);
      }
    } else {
      throw std::runtime_error("zlib error: " + std::to_string(state));
    }
  }

  return infstream.total_out;
}
