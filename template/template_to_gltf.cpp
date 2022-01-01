/*  TMP2GLTF
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
#include "datas/except.hpp"
#include "datas/fileinfo.hpp"
#include "datas/master_printer.hpp"
#include "datas/reflector.hpp"
#include "inf_gltf.hpp"
#include "project.h"
#include <vector>

es::string_view filters[]{
    ".tmpdata$",
    {},
};

struct XSIBIN2GLTF : ReflectorBase<XSIBIN2GLTF> {
} settings;

REFLECT(CLASS(XSIBIN2GLTF));

AppInfo_s appInfo{
    AppInfo_s::CONTEXT_VERSION,
    AppMode_e::CONVERT,
    ArchiveLoadType::ALL,
    TMP2GLTF_DESC " v" TMP2GLTF_VERSION ", " TMP2GLTF_COPYRIGHT "Lukas Cone",
    reinterpret_cast<ReflectorFriend *>(&settings),
    filters,
};

const AppInfo_s *AppInitModule() { return &appInfo; }

MAKE_ENUM(ENUMSCOPE(class Type, Type), EMEMBER(None), EMEMBER(Template),
          EMEMBER(Animations), EMEMBER(Meshes), EMEMBER(Materials));

struct Animation {
  std::string name;
  std::string file;
};

void AppProcessFile(std::istream &stream, AppContext *ctx) {
  std::string line;
  std::getline(stream, line);

  auto Trim = [](auto sv) { return es::TrimWhitespace(sv, true); };

  if (auto sline = Trim(es::string_view(line)); sline != "template") {
    throw es::InvalidHeaderError(sline);
  }

  Type curType = Type::Template;
  Animation curAnim;
  std::vector<Animation> animations;
  AFileInfo mesh;

  while (!std::getline(stream, line).eof()) {
    auto trimmedLine = Trim(es::string_view(line));

    switch (curType) {
    case Type::None: {
      if (trimmedLine == "animations") {
        curType = Type::Animations;
      } else if (trimmedLine == "meshes") {
        curType = Type::Meshes;
      } else if (trimmedLine == "material-params") {
        curType = Type::Materials;
      } else if (!trimmedLine.empty()) {
        printwarning("Undefined section: " << trimmedLine);
        curType = Type::None;
      }
      break;
    }
    case Type::Template: {
      if (trimmedLine == "end-variables") {
        curType = Type::None;
      }
      break;
    }

    case Type::Animations: {
      if (trimmedLine == "end-animations") {
        curType = Type::None;
      } else if (trimmedLine.begins_with("File")) {
        trimmedLine.remove_prefix(5);
        curAnim.file = Trim(trimmedLine);
      } else if (trimmedLine.begins_with("Name")) {
        trimmedLine.remove_prefix(5);
        curAnim.name = Trim(trimmedLine);
      } else if (trimmedLine == "end-slot") {
        animations.emplace_back(std::move(curAnim));
      }
      break;
    }

    case Type::Meshes: {
      if (trimmedLine == "meshes-end") {
        curType = Type::None;
      } else if (trimmedLine.begins_with("file")) {
        trimmedLine.remove_prefix(5);
        if (!mesh.GetFullPath().empty()) {
          printerror("More than one mesh found, overriding.");
        }
        mesh.Load(Trim(trimmedLine));
      }
      break;
    }

    case Type::Materials: {
      if (trimmedLine == "end-material-params") {
        curType = Type::None;
      }
      break;
    }

    default:
      break;
    }
  }

  if (curType != Type::None) {
    printerror("Ending was not found for block: "
               << GetReflectedEnum<Type>()->names[size_t(curType)]);
  }

  if (mesh.GetFullPath().empty()) {
    printerror("Template doesn't contain mesh, skipping.");
    return;
  }

  auto meshPath = mesh.GetFullPathNoExt().to_string() + ".xbn$";
  AFileInfo fInfo(ctx->workingFile);
  auto workingDir = fInfo.GetFolder().to_string();
  auto foundMesh = ctx->FindFile(workingDir, meshPath);

  InfGLTF main;
  main.AddXBN(*foundMesh.Get());

  for (auto &a : animations)
    try {
      AFileInfo correctFile(a.file);
      auto foundAnim = ctx->FindFile(
          workingDir, correctFile.GetFilenameExt().to_string() + ".bin$");
      main.AddAnimation(*foundAnim.Get(), a.name);
    } catch (const std::exception &e) {
      printerror(e.what());
    }

  main.FinishAndSave(fInfo.GetFullPathNoExt().to_string() + ".glb");
}
