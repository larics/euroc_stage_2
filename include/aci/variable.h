/*
* Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
* You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef ACI_VARIABLE_H_
#define ACI_VARIABLE_H_

#include <memory>

#include <aci/implementation/asctecDefines.h>

#if ACI_VER_MAJOR < 2
extern "C" {

union VECT3I {
  struct {
    int x, y, z;
  };
  int elem[3];
};
typedef union VECT3I vector3i;

union VECT2I {
  struct {
    int x, y;
  };
  int elem[2];
};
typedef union VECT2I vector2i;

union QUATERNION {
  struct {
    float q1, q2, q3, q4;
  };
  struct {
    float w, x, y, z;
  };
  float elem[4];
};
typedef union QUATERNION quaternion;

union VECT3F {
  struct {
    float x, y, z;
  };
  float elem[3];
};
typedef union VECT3F vector3f;

union VECT2F {
  struct {
    float x, y;
  };
  float elem[2];
};
typedef union VECT2F vector2f;
}

/// Computes the size of the asctec variable type at compile time.
constexpr size_t getAsctecVariableSize(int asctec_variable_type) {
  return 0x3f & (asctec_variable_type >> 2);
}

static constexpr size_t kAsctecMaximumVariableSize = 1 << (sizeof(char) * 8 - 2);

#else

/// Computes the size of the asctec variable type at compile time.
constexpr size_t getAsctecVariableSize(int asctec_variable_type) {
  return 0x1fff & (AsctecVariableDefinition_ >> 3); // assuming it's an unsigned short now
}

static constexpr size_t kAsctecMaximumVariableSize = 1 << (sizeof(short) * 8 - 3);

#endif


namespace aci {

template<int AsctecVariableDefinition_>
struct TypifyAsctecVariableDefinition {
  typedef void* Type;
  static constexpr const char* Description = "void*";
  static constexpr int AsctecVariableType = AsctecVariableDefinition_;
  static constexpr size_t Size = getAsctecVariableSize(AsctecVariableDefinition_);

  static_assert( AsctecVariableDefinition_ != AsctecVariableDefinition_, "Specialization not found");
};

#define ACI_SPECIALIZE_VARIABLE_TYPE_INFO(aci_type, storage_type, variable_name) \
template <> \
struct TypifyAsctecVariableDefinition<aci_type>{ \
  typedef storage_type Type; \
  static constexpr const char* Description = #storage_type; \
  static constexpr int AsctecVariableType = aci_type; \
  static constexpr size_t Size = getAsctecVariableSize(aci_type); \
}; \
typedef TypifyAsctecVariableDefinition<aci_type> variable_name; \

ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_DOUBLE, double, VariableDouble)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_SINGLE, float, VariableFloat)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_INT16, int16_t, VariableInt16)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_INT32, int32_t, VariableInt32)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_INT64, int64_t, VariableInt64)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_INT8, int8_t, VariableInt8)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_UINT8, uint8_t, VariableUint8)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_UINT16, uint16_t, VariableUint16)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_UINT32, uint32_t, VariableUint32)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_UINT64, uint64_t, VariableUint64)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_QUAT, quaternion, VariableQuaternion)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_VECTOR_3F, vector3f, VariableVector3f)
ACI_SPECIALIZE_VARIABLE_TYPE_INFO(VARTYPE_VECTOR_3I, vector3i, VariableVector3i)

class VariableBase {
 public:
  typedef std::shared_ptr<VariableBase> Ptr;
  VariableBase(int id, int type)
      : id_(id),
        type_(type) {
  }

  virtual ~VariableBase() {
  }

  virtual void* getRawPtr() const = 0;

  int getType() const {
    return type_;
  }

  int getId() const {
    return id_;
  }

 private:
  int id_;
  int type_;
};

template<class CustomObject_>
class Variable : public VariableBase {
 public:
  typedef CustomObject_ Type;
  static constexpr const char* Description = "custom object";
  static constexpr int AsctecVariableType = VARTYPE_STRUCT_WITH_SIZE(sizeof(CustomObject_));
  static constexpr size_t Size = sizeof(CustomObject_);

  static_assert(Size <= kAsctecMaximumVariableSize, "size of struct too large");

  Variable(int id = -1)
      : VariableBase(id, AsctecVariableType) {
    data_ = std::shared_ptr<CustomObject_>(new CustomObject_);
  }

  Variable<CustomObject_>& operator=(const CustomObject_& value) {
    *data_ = value;
    return *this;
  }

  void* getRawPtr() const {
    return reinterpret_cast<void*>(data_.get());
  }

  std::shared_ptr<CustomObject_> getPtr(){
    return data_;
  }

  CustomObject_ value() const {
    return *data_;
  }

 private:
  std::shared_ptr<CustomObject_> data_;
};

template<int AsctecVariableType_>
class Variable<TypifyAsctecVariableDefinition<AsctecVariableType_> > : public VariableBase {
 public:
  typedef typename TypifyAsctecVariableDefinition<AsctecVariableType_>::Type Type;
  static constexpr const char* Description = TypifyAsctecVariableDefinition<AsctecVariableType_>::Description;
  static constexpr int AsctecVariableType = AsctecVariableType_;
  static constexpr size_t Size = TypifyAsctecVariableDefinition<AsctecVariableType_>::Size;

  Variable(int id = -1)
      : VariableBase(id, AsctecVariableType) {
    data_.reset(new Type);
  }

  Variable<TypifyAsctecVariableDefinition<AsctecVariableType_> >& operator=(const Type& value) {
    *data_ = value;
    return *this;
  }

  void* getRawPtr() const {
    return reinterpret_cast<void*>(data_.get());
  }

  std::shared_ptr<Type> getPtr(){
    return data_;
  }

  Type value() const {
    return *data_;
  }

 private:
  std::shared_ptr<Type> data_;
};

} // end namespace aci

#endif /* ACI_COMMAND_H_ */
