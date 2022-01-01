/*  XBN2GLTF
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
#include "datas/fileinfo.hpp"
#include "datas/reflector.hpp"
#include "inf_gltf.hpp"
#include "project.h"

es::string_view filters[]{
    ".xbn$",
    {},
};

struct XBN2GLTF : ReflectorBase<XBN2GLTF> {
} settings;

REFLECT(CLASS(XBN2GLTF));

AppInfo_s appInfo{
    AppInfo_s::CONTEXT_VERSION,
    AppMode_e::CONVERT,
    ArchiveLoadType::FILTERED,
    XBN2GLTF_DESC " v" XBN2GLTF_VERSION ", " XBN2GLTF_COPYRIGHT "Lukas Cone",
    reinterpret_cast<ReflectorFriend *>(&settings),
    filters,
};

const AppInfo_s *AppInitModule() { return &appInfo; }

using namespace fx;

void AppProcessFile(std::istream &stream, AppContext *ctx) {
  InfGLTF main;
  main.AddXBN(stream);
  AFileInfo fInfo(ctx->workingFile);
  main.FinishAndSave(fInfo.GetFullPathNoExt().to_string() + ".glb");
}
