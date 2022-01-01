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
#include "datas/binreader.hpp"
#include "datas/fileinfo.hpp"
#include "datas/master_printer.hpp"
#include "datas/reflector.hpp"
#include "inf_gltf.hpp"
#include "project.h"

es::string_view filters[]{
    ".xsi.bin$",
    {},
};

struct XSIBIN2GLTF : ReflectorBase<XSIBIN2GLTF> {
  std::string skeleton;
} settings;

REFLECT(CLASS(XSIBIN2GLTF),
        MEMBERNAME(skeleton, "skeleton-file",
                   ReflDesc{"Load skeleton data from xbn model file."}));

AppInfo_s appInfo{
    AppInfo_s::CONTEXT_VERSION,
    AppMode_e::CONVERT,
    ArchiveLoadType::FILTERED,
    XSIBIN2GLTF_DESC " v" XSIBIN2GLTF_VERSION ", " XSIBIN2GLTF_COPYRIGHT
                     "Lukas Cone",
    reinterpret_cast<ReflectorFriend *>(&settings),
    filters,
};

const AppInfo_s *AppInitModule() { return &appInfo; }

void AppProcessFile(std::istream &stream, AppContext *ctx) {
  InfGLTF main;
  AFileInfo fInfo(ctx->workingFile);

  if (!settings.skeleton.empty())
    [&] {
      BinReader<> rdm;
      BinReaderRef rd;
      AppContextStream str;

      try {
        rdm.Open(settings.skeleton);
        rd = rdm;
      } catch (...) {
        try {
          str = ctx->RequestFile(settings.skeleton);
          rd = *str.Get();
        } catch (...) {
          try {
            str = ctx->FindFile(fInfo.GetFolder(), settings.skeleton);
            rd = *str.Get();
          } catch (...) {
            printerror("Couldn't open skeleton file: " << settings.skeleton);
            return;
          }
        }
      }

      main.AddXBN(rd, true);
    }();

  main.AddAnimation(stream, {});

  main.FinishAndSave(fInfo.GetFullPathNoExt().to_string() + ".glb");
}
