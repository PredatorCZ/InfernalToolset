/*  TEX2DDS
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

#include "datas/app_context.hpp"
#include "datas/binreader_stream.hpp"
#include "datas/binwritter.hpp"
#include "datas/except.hpp"
#include "datas/fileinfo.hpp"
#include "datas/reflector.hpp"
#include "formats/DDS.hpp"
#include "project.h"
#include <vector>
#include <cmath>

es::string_view filters[]{
    ".texture$",
    {},
};

struct TEX2DDS : ReflectorBase<TEX2DDS> {
} settings;

REFLECT(CLASS(TEX2DDS));

AppInfo_s appInfo{
    AppInfo_s::CONTEXT_VERSION,
    AppMode_e::CONVERT,
    ArchiveLoadType::FILTERED,
    TEX2DDS_DESC " v" TEX2DDS_VERSION ", " TEX2DDS_COPYRIGHT "Lukas Cone",
    reinterpret_cast<ReflectorFriend *>(&settings),
    filters,
};

const AppInfo_s *AppInitModule() { return &appInfo; }

struct Header {
  static constexpr uint32 ID = CompileFourCC("S3TC");
  uint32 id;
  uint32 fourcc;
  uint32 width;
  uint32 height;
  uint32 dataSize;
  uint16 format;
  bool cubemap;
  bool xboxNative;
  uint32 null[2];
};

void AppProcessFile(std::istream &stream, AppContext *ctx) {
  BinReaderRef rd(stream);
  Header hdr;
  rd.Read(hdr);
  rd.Skip(1);

  if (hdr.id != Header::ID) {
    throw es::InvalidHeaderError(hdr.id);
  }

  if (hdr.xboxNative) {
    throw std::runtime_error("Xbox native texture data not supported.");
  }

  DDS main;
  main.width = hdr.width;
  main.height = hdr.height;
  main.fourCC = hdr.fourcc;
  main.pfFlags += DDS_PixelFormat::PFFlags_FourCC;
  main.NumMipmaps(log2(std::max(hdr.width, hdr.height)) + 1);

  if (hdr.cubemap) {
      main.caps01 += DDS_HeaderEnd::Caps01Flags_CubeMap;
      main.caps01 += DDS_HeaderEnd::Caps01Flags_CubeMap_NegativeX;
      main.caps01 += DDS_HeaderEnd::Caps01Flags_CubeMap_NegativeY;
      main.caps01 += DDS_HeaderEnd::Caps01Flags_CubeMap_NegativeZ;
      main.caps01 += DDS_HeaderEnd::Caps01Flags_CubeMap_PositiveX;
      main.caps01 += DDS_HeaderEnd::Caps01Flags_CubeMap_PositiveY;
      main.caps01 += DDS_HeaderEnd::Caps01Flags_CubeMap_PositiveZ;
  }

  AFileInfo fInfo(ctx->workingFile);

  BinWritter wr(fInfo.GetFullPathNoExt().to_string() + ".dds");
  wr.WriteBuffer(reinterpret_cast<const char *>(&main), DDS::LEGACY_SIZE);

  std::string buff;
  rd.ReadContainer(buff, hdr.dataSize);
  wr.WriteContainer(buff);
}
