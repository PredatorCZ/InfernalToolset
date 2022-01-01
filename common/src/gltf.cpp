/*  Infernal gltf shared methods
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

#include "gltf.hpp"
#include "datas/binwritter.hpp"
#include "datas/except.hpp"
#include "datas/fileinfo.hpp"
#include "datas/matrix44.hpp"
#include "datas/vectors.hpp"
#include "inf_gltf.hpp"
#include "xbn.hpp"
#include <sstream>
#include <vector>

#include "glm/ext.hpp"
#include "glm/glm.hpp"

class InfGLTFImpl : public GLTF {
  int32 meshBeginIndex = -1;
  int32 weightBeginIndex = -1;
  int32 secondUVIndex = -1;

public:
  GLTFStream &MeshStream(size_t offset = 0) {
    return Stream(meshBeginIndex + offset);
  }
  GLTFStream &SkinStream(size_t offset = 0) {
    return Stream(weightBeginIndex + offset);
  }
  GLTFStream &UV2Stream() { return Stream(secondUVIndex); }

  void MakeBaseMeshStreams() {
    if (meshBeginIndex >= 0) {
      return;
    }
    meshBeginIndex = NumStreams();
    auto &indices = NewStream("indices");
    indices.target = gltf::BufferView::TargetType::ElementArrayBuffer;
    auto &vertices = NewStream("vertices", 32);
    vertices.target = gltf::BufferView::TargetType::ArrayBuffer;
  }

  void MakeMeshSkinStreams() {
    if (weightBeginIndex >= 0) {
      return;
    }

    weightBeginIndex = NumStreams();
    auto &weights = NewStream("weights", 8);
    weights.target = gltf::BufferView::TargetType::ArrayBuffer;

    NewStream("skins");
  }

  void MakeSecondaryUVs() {
    if (secondUVIndex >= 0) {
      return;
    }

    secondUVIndex = NumStreams();
    auto &uvs = NewStream("UV2");
    uvs.target = gltf::BufferView::TargetType::ArrayBuffer;
  }

  void AddXBN(BinReaderRef rd, bool onlySkeleton);
  void AddAnimation(BinReaderRef rd, const std::string &name);

  auto NewAnim(const std::string &name) {
    auto &ani = animations.emplace_back();
    ani.name = name;
    auto &anistr = NewStream(name.empty() ? "animation" : name + "-data");

    return std::make_pair(std::ref(ani), std::ref(anistr));
  }

  size_t MakeSingleFrame() {
    auto [acc, singleFrameAccessor] = NewAccessor(LastStream(), 4);
    acc.componentType = gltf::Accessor::ComponentType::Float;
    acc.type = gltf::Accessor::Type::Scalar;
    acc.count = 1;
    acc.min.push_back(0);
    acc.max.push_back(0);
    LastStream().wr.Write(0.f);

    return singleFrameAccessor;
  }
};

void InfGLTFImpl::AddXBN(BinReaderRef rd, bool onlySkeleton) {
  Header hdr;
  rd.Read(hdr);
  gltf::Primitive prim;

  if (!onlySkeleton) {
    {
      MakeBaseMeshStreams();
      std::vector<Vertex> verts;
      rd.ReadContainer(verts, hdr.numVerts);

      gltf::Accessor baseAcc;
      baseAcc.componentType = gltf::Accessor::ComponentType::Float;
      baseAcc.type = gltf::Accessor::Type::Vec3;
      baseAcc.count = hdr.numVerts;

      auto [posAcc, posId] = NewAccessor(MeshStream(1), 4, 0, &baseAcc);
      prim.attributes["POSITION"] = posId;
      Vector maxPos(-FLT_MAX);
      Vector minPos(FLT_MAX);

      for (size_t e = 0; e < 3; e++) {
        for (auto &v : verts) {
          if (v.pos[e] > maxPos[e]) {
            maxPos[e] = v.pos[e];
          }

          if (v.pos[e] < minPos[e]) {
            minPos[e] = v.pos[e];
          }
        }
      }
      posAcc.max.resize(3);
      posAcc.min.resize(3);
      memcpy(posAcc.max.data(), &maxPos, sizeof(Vector));
      memcpy(posAcc.min.data(), &minPos, sizeof(Vector));

      prim.attributes["NORMAL"] =
          NewAccessor(MeshStream(1), 4, 12, &baseAcc).second;

      auto [uvSet, uvId] = NewAccessor(MeshStream(1), 4, 24, &baseAcc);
      uvSet.type = gltf::Accessor::Type::Vec2;
      prim.attributes["TEXCOORD_0"] = uvId;

      MeshStream(1).wr.WriteContainer(verts);
    }

    if (hdr.useUV2) {
      MakeSecondaryUVs();
      std::vector<Vector2> lmuv;
      rd.ReadContainer(lmuv, hdr.numVerts);
      auto [baseAcc, baseId] = NewAccessor(UV2Stream(), 4);
      baseAcc.componentType = gltf::Accessor::ComponentType::Float;
      baseAcc.type = gltf::Accessor::Type::Vec2;
      baseAcc.count = hdr.numVerts;
      prim.attributes["TEXCOORD_1"] = baseId;
      UV2Stream().wr.WriteContainer(lmuv);
    }

    {
      std::vector<uint16> indices;
      rd.ReadContainer(indices, hdr.numFaces * 3);
      MeshStream().wr.WriteContainer(indices);

      /*for (auto i : indices) {
        if (i > hdr.numVerts) {
          printinfo("Corrupted vertices!");
          break;
        }
      }*/
    }
  } else {
    size_t skipValue =
        rd.Tell() + (sizeof(Vertex) * hdr.numVerts) + (6 * hdr.numFaces);

    if (hdr.useUV2) {
      skipValue += 8 * hdr.numVerts;
    }

    rd.Seek(skipValue);
  }

  { // unused
    std::vector<uint8> trisFlags;
    rd.ReadContainer(trisFlags);

    std::vector<PhysType> physTypes;

    if (!trisFlags.empty()) {
      rd.ReadContainer(physTypes);
    }

    std::vector<USVector2> edges; //?
    rd.ReadContainer(edges);
  }

  std::vector<Bone> bones;
  rd.ReadContainer(bones, hdr.numNodes);

  if (onlySkeleton) {
    for (auto &b : bones) {
      const size_t boneId = nodes.size();

      nodes.emplace_back();
      auto &mNode = nodes.back();
      mNode.name = b.boneName;
      memcpy(mNode.matrix.data(), &b.transform, sizeof(es::Matrix44));

      /*
      Vector4A16 cPos, cRot, cScale;
      b.transform.Decompose(cPos, cRot, cScale);
      memcpy(mNode.translation.data(), &cPos, 12);
      memcpy(mNode.scale.data(), &cScale, 12);
      memcpy(mNode.rotation.data(), &cRot, 12);
      */

      if (b.parentId >= 0) {
        nodes.at(b.parentId).children.push_back(boneId);
      }
    }

    return;
  }

  std::vector<Material> materials;
  rd.ReadContainer(materials, hdr.numTextures);

  uint32 maxSkinBones = 0;

  {
    std::vector<uint32> weightData;
    rd.ReadContainer(weightData);

    std::vector<uint32> weightIndices;
    rd.ReadContainer(weightIndices);
    rd.Read(maxSkinBones);
    // note, some data follows (collision vertices?)

    if (!weightIndices.empty()) {
      MakeMeshSkinStreams();

      auto [jointAccess, jointId] = NewAccessor(SkinStream(), 4);
      jointAccess.componentType = gltf::Accessor::ComponentType::UnsignedByte;
      jointAccess.type = gltf::Accessor::Type::Vec4;
      jointAccess.count = hdr.numVerts;
      prim.attributes["JOINTS_0"] = jointId;

      auto [wtAccess, wtId] = NewAccessor(SkinStream(), 4, 4);
      wtAccess.componentType = gltf::Accessor::ComponentType::UnsignedByte;
      wtAccess.normalized = true;
      wtAccess.type = gltf::Accessor::Type::Vec4;
      wtAccess.count = hdr.numVerts;
      prim.attributes["WEIGHTS_0"] = wtId;

      for (auto w : weightIndices) {
        uint8 numWeights = w >> 24;
        uint32 weightIndex = w & 0xffffff;
        uint8 buffer[8]{};
        Vector4A16 weights;

        for (uint32 i = 0; i < numWeights; i++) {
          uint32 wt = weightData.at(weightIndex + i);
          weights[i] = (wt >> 8) * (1.f / 0xffffff);
          buffer[i] = wt & 0xff;
        }

        const float suma =
            1.f / (weights.X + weights.Y + weights.Z + weights.W);
        weights *= suma * 0xff;
        weights = Vector4A16(_mm_round_ps(
            weights._data, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
        weights = Vector4A16(_mm_min_ps(weights._data, Vector4A16(255)._data));
        const float restSuma =
            0xff - (weights.X + weights.Y + weights.Z + weights.W);

        if (restSuma) {
          weights[0] += restSuma + 0.99f;
        }

        for (uint32 i = 0; i < numWeights; i++) {
          buffer[i + 4] = weights[i];
        }

        SkinStream().wr.Write(buffer);
      }
    }
  }

  gltf::Mesh mesh;
  {
    size_t lastFace = 0;

    for (auto &m : materials) {
      size_t numFaces = m.endFace - lastFace;

      auto [indexAccess, indexId] = NewAccessor(MeshStream(), 2);
      indexAccess.componentType = gltf::Accessor::ComponentType::UnsignedShort;
      indexAccess.count = numFaces * 3;
      indexAccess.byteOffset = lastFace * 6;
      indexAccess.type = gltf::Accessor::Type::Scalar;

      auto &nPrim = mesh.primitives.emplace_back(prim);
      nPrim.indices = indexId;
      nPrim.material = this->materials.size();

      gltf::Material material;
      material.name = m.textureName;
      this->materials.emplace_back(std::move(material));

      lastFace = m.endFace;
    }
  }

  if (!bones.empty()) {
    scenes.front().nodes.push_back(0);
    auto &skin = skins.emplace_back();
    skin.joints.resize(maxSkinBones);
    std::vector<es::Matrix44> ibms;
    ibms.resize(maxSkinBones);

    for (auto &b : bones) {
      const size_t boneId = nodes.size();

      if (b.skinBoneId >= 0) {
        skin.joints.at(b.skinBoneId) = boneId;
        ibms.at(b.skinBoneId) = b.ibm;
      }

      nodes.emplace_back();
      auto &mNode = nodes.back();
      mNode.name = b.boneName;
      memcpy(mNode.matrix.data(), &b.transform, sizeof(es::Matrix44));

      /*
      Vector4A16 cPos, cRot, cScale;
      b.transform.Decompose(cPos, cRot, cScale);
      memcpy(mNode.translation.data(), &cPos, 12);
      memcpy(mNode.scale.data(), &cScale, 12);
      memcpy(mNode.rotation.data(), &cRot, 12);
      */

      if (b.parentId >= 0) {
        nodes.at(b.parentId).children.push_back(boneId);
      }
    }

    auto [ibmAccess, ibmId] = NewAccessor(SkinStream(1), 16);
    ibmAccess.componentType = gltf::Accessor::ComponentType::Float;
    ibmAccess.count = ibms.size();
    ibmAccess.type = gltf::Accessor::Type::Mat4;
    skin.inverseBindMatrices = ibmId;
    SkinStream(1).wr.WriteContainer(ibms);
  }

  scenes.front().nodes.push_back(nodes.size());
  nodes.emplace_back();
  auto &mNode = nodes.back();
  mNode.mesh = meshes.size();

  if (!bones.empty()) {
    mNode.skin = skins.size() - 1;
  }

  meshes.emplace_back(std::move(mesh));
}

struct Frame {
  float time;
  Vector value;
};

struct Node {
  uint16 numPosTracks;
  uint16 numRotTracks;
};

struct NodeEx {
  glm::quat rotation;
  Vector pos;
  int32 index = -1;
};

void InfGLTFImpl::AddAnimation(BinReaderRef rd, const std::string &name) {
  char idbuffer[8];
  rd.Read(idbuffer);
  es::string_view id(idbuffer);

  if (id != "XSIBIN ") {
    throw es::InvalidHeaderError(id);
  }

  uint8 version;
  rd.Read(version);

  if (version != 7) {
    throw es::InvalidVersionError(version);
  }

  uint32 null;
  rd.Read(null);

  uint32 numBones;
  rd.Read(numBones);

  float duration;
  rd.Read(duration);

  std::vector<Node> nodes;
  std::vector<NodeEx> usedNodes;
  nodes.resize(numBones);
  const bool freeLoad = nodes.empty();

  for (size_t i = 0; i < numBones; i++) {
    std::string nodeName;
    rd.ReadContainer<uint8>(nodeName);
    std::transform(nodeName.begin(), nodeName.end(), nodeName.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (freeLoad) {
      NodeEx &nodeEx = usedNodes.emplace_back();
      nodeEx.index = nodes.size();
      this->nodes.emplace_back().name = nodeName;
      continue;
    }

    auto found =
        std::find_if(this->nodes.begin(), this->nodes.end(),
                     [&](auto &item) { return item.name == nodeName; });

    if (!es::IsEnd(this->nodes, found)) {
      NodeEx &nodeEx = usedNodes.emplace_back();
      nodeEx.index = std::distance(this->nodes.begin(), found);
      es::Matrix44 mtx;
      memcpy(&mtx, found->matrix.data(), sizeof(mtx));
      Vector4A16 cPos, sScale, cRot;
      mtx.Decompose(cPos, cRot, sScale);
      nodeEx.pos = cPos;
      nodeEx.rotation = glm::quat(cRot.W, cRot.X, cRot.Y, cRot.Z);
    } else {
      usedNodes.push_back({});
    }

    rd.Read(nodes.at(i));
  }

  auto [ani, aniStr] = NewAnim(name);
  auto singleFrameAccessor = MakeSingleFrame();

  auto MakeInputAccessor = [&, &aniStr = aniStr](size_t numFrames) {
    auto [timeAccess, index] = NewAccessor(aniStr, 4);
    timeAccess.componentType = gltf::Accessor::ComponentType::Float;
    timeAccess.type = gltf::Accessor::Type::Scalar;
    timeAccess.count = numFrames;
    timeAccess.min.push_back(0);
    timeAccess.max.push_back(duration);

    return index;
  };

  auto GetInputAccessorIndex = [&](size_t numFrames) {
    if (numFrames == 1) {
      return singleFrameAccessor;
    }

    return MakeInputAccessor(numFrames);
  };

  for (size_t i = 0; i < numBones; i++) {
    auto &node = nodes.at(i);
    auto &nodeEx = usedNodes.at(i);

    std::vector<Frame> rotFrames;
    rd.ReadContainer(rotFrames, node.numRotTracks);

    if (nodeEx.index >= 0) {
      auto &rChannel = ani.channels.emplace_back();
      rChannel.target.path = "rotation";
      rChannel.target.node = nodeEx.index;
      rChannel.sampler = ani.samplers.size();

      auto &rSampler = ani.samplers.emplace_back();
      rSampler.input = GetInputAccessorIndex(node.numRotTracks);

      if (node.numRotTracks > 1) {
        for (auto &f : rotFrames) {
          aniStr.wr.Write(f.time);
        }
      }

      auto [rotAccess, rotId] = NewAccessor(aniStr, 2);
      rotAccess.componentType = gltf::Accessor::ComponentType::Short;
      rotAccess.normalized = true;
      rotAccess.type = gltf::Accessor::Type::Vec4;
      rotAccess.count = node.numRotTracks;
      rSampler.output = rotId;

      for (auto &f : rotFrames) {
        glm::quat quat(
            glm::radians(reinterpret_cast<const glm::vec3 &>(f.value)));
        quat = nodeEx.rotation * quat;

        Vector4A16 rot(quat.x, quat.y, quat.z, quat.w);
        aniStr.wr.Write((rot * 0x7fff).Convert<int16>());
      }
    }

    es::Dispose(rotFrames);

    std::vector<Frame> posFrames;
    rd.ReadContainer(posFrames, node.numPosTracks);

    if (nodeEx.index >= 0) {
      auto &pChannel = ani.channels.emplace_back();
      pChannel.target.path = "translation";
      pChannel.target.node = nodeEx.index;
      pChannel.sampler = ani.samplers.size();

      auto &pSampler = ani.samplers.emplace_back();
      pSampler.input = GetInputAccessorIndex(node.numPosTracks);

      if (node.numRotTracks > 1) {
        for (auto &f : rotFrames) {
          aniStr.wr.Write(f.time);
        }
      }

      auto [posAccess, posId] = NewAccessor(aniStr, 4);
      posAccess.componentType = gltf::Accessor::ComponentType::Float;
      posAccess.type = gltf::Accessor::Type::Vec3;
      posAccess.count = node.numPosTracks;
      pSampler.output = posId;

      for (auto &f : posFrames) {
        aniStr.wr.Write(f.value + nodeEx.pos);
      }
    }
  }
}

InfGLTF::InfGLTF() : pi(std::make_unique<InfGLTFImpl>()) {}
InfGLTF::~InfGLTF() = default;
void InfGLTF::AddXBN(BinReaderRef rd, bool onlySkeleton) {
  pi->AddXBN(rd, onlySkeleton);
}

void InfGLTF::FinishAndSave(const std::string &path) {
  BinWritter wr(path);
  AFileInfo info(path);
  pi->FinishAndSave(wr, info.GetFolder());
}

void InfGLTF::AddAnimation(BinReaderRef rd, const std::string &name) {
  pi->AddAnimation(rd, name);
}
