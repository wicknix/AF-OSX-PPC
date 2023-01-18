/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "WebGL2Context.h"
#include "GLContext.h"
#include "WebGLContext.h"
#include "WebGLProgram.h"
#include "WebGLVertexArray.h"
#include "WebGLVertexAttribData.h"
#include "mozilla/dom/WebGL2RenderingContextBinding.h"

using namespace mozilla;
using namespace mozilla::dom;

typedef union { GLint i; GLfloat f; GLuint u; } fi_t;

static inline
GLfloat PuntToFloat(GLint i)
{
   fi_t tmp;
   tmp.i = i;
   return tmp.f;
}

static inline
GLfloat PuntToFloat(GLuint u)
{
   fi_t tmp;
   tmp.u = u;
   return tmp.f;
}

bool
WebGL2Context::ValidateAttribPointerType(bool integerMode, GLenum type,
                                         GLsizei* out_alignment, const char* info)
{
    MOZ_ASSERT(out_alignment);

    switch (type) {
    case LOCAL_GL_BYTE:
    case LOCAL_GL_UNSIGNED_BYTE:
        *out_alignment = 1;
        return true;

    case LOCAL_GL_SHORT:
    case LOCAL_GL_UNSIGNED_SHORT:
        *out_alignment = 2;
        return true;

    case LOCAL_GL_INT:
    case LOCAL_GL_UNSIGNED_INT:
        *out_alignment = 4;
        return true;
    }

    if (!integerMode) {
        switch (type) {
        case LOCAL_GL_HALF_FLOAT:
            *out_alignment = 2;
            return true;

        case LOCAL_GL_FLOAT:
        case LOCAL_GL_FIXED:
        case LOCAL_GL_INT_2_10_10_10_REV:
        case LOCAL_GL_UNSIGNED_INT_2_10_10_10_REV:
            *out_alignment = 4;
            return true;
        }
    }

    ErrorInvalidEnum("%s: invalid enum value 0x%x", info, type);
    return false;
}

// -------------------------------------------------------------------------
// Uniforms and attributes

void
WebGL2Context::VertexAttribIPointer(GLuint index, GLint size, GLenum type, GLsizei stride,
                                    GLintptr offset)
{
    if (IsContextLost())
        return;

    if (!ValidateAttribIndex(index, "vertexAttribIPointer"))
        return;

    if (!ValidateAttribPointer(true, index, size, type, LOCAL_GL_FALSE, stride, offset,
                               "vertexAttribIPointer"))
    {
        return;
    }

    MOZ_ASSERT(mBoundVertexArray);
    mBoundVertexArray->EnsureAttrib(index);

    InvalidateBufferFetching();

    WebGLVertexAttribData& vd = mBoundVertexArray->mAttribs[index];

    vd.buf = mBoundArrayBuffer;
    vd.stride = stride;
    vd.size = size;
    vd.byteOffset = offset;
    vd.type = type;
    vd.normalized = false;
    vd.integer = true;

    MakeContextCurrent();
    gl->fVertexAttribIPointer(index, size, type, stride, reinterpret_cast<void*>(offset));
}

void
WebGL2Context::Uniform1ui(WebGLUniformLocation* location, GLuint v0)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::Uniform2ui(WebGLUniformLocation* location, GLuint v0, GLuint v1)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::Uniform3ui(WebGLUniformLocation* location, GLuint v0, GLuint v1, GLuint v2)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::Uniform4ui(WebGLUniformLocation* location, GLuint v0, GLuint v1,
                          GLuint v2, GLuint v3)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::Uniform1uiv(WebGLUniformLocation* location,
                           const dom::Sequence<GLuint>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::Uniform2uiv(WebGLUniformLocation* location,
                           const dom::Sequence<GLuint>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::Uniform3uiv(WebGLUniformLocation* location,
                           const dom::Sequence<GLuint>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::Uniform4uiv(WebGLUniformLocation* location,
                           const dom::Sequence<GLuint>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix2x3fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Float32Array& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix2x3fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Sequence<GLfloat>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix3x2fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Float32Array& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix3x2fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Sequence<GLfloat>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix2x4fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Float32Array& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix2x4fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Sequence<GLfloat>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix4x2fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Float32Array& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix4x2fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Sequence<GLfloat>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix3x4fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Float32Array& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix3x4fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Sequence<GLfloat>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix4x3fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Float32Array& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::UniformMatrix4x3fv(WebGLUniformLocation* location, bool transpose,
                                  const dom::Sequence<GLfloat>& value)
{
    MOZ_CRASH("Not Implemented.");
}

void
WebGL2Context::VertexAttribI4i(GLuint index, GLint x, GLint y, GLint z, GLint w)
{
    if (IsContextLost())
        return;

    if (index || gl->IsGLES()) {
        MakeContextCurrent();
        gl->fVertexAttribI4i(index, x, y, z, w);
    } else {
        mVertexAttrib0Vector[0] = PuntToFloat(x);
        mVertexAttrib0Vector[1] = PuntToFloat(y);
        mVertexAttrib0Vector[2] = PuntToFloat(z);
        mVertexAttrib0Vector[3] = PuntToFloat(w);
    }
}

void
WebGL2Context::VertexAttribI4iv(GLuint index, size_t length, const GLint* v)
{
    if (!ValidateAttribArraySetter("vertexAttribI4iv", 4, length))
        return;

    if (index || gl->IsGLES()) {
        MakeContextCurrent();
        gl->fVertexAttribI4iv(index, v);
    } else {
        mVertexAttrib0Vector[0] = PuntToFloat(v[0]);
        mVertexAttrib0Vector[1] = PuntToFloat(v[1]);
        mVertexAttrib0Vector[2] = PuntToFloat(v[2]);
        mVertexAttrib0Vector[3] = PuntToFloat(v[3]);
    }
}

void
WebGL2Context::VertexAttribI4iv(GLuint index, const dom::Sequence<GLint>& v)
{
    VertexAttribI4iv(index, v.Length(), v.Elements());
}

void
WebGL2Context::VertexAttribI4ui(GLuint index, GLuint x, GLuint y, GLuint z, GLuint w)
{
    if (IsContextLost())
        return;

    if (index || gl->IsGLES()) {
        MakeContextCurrent();
        gl->fVertexAttribI4ui(index, x, y, z, w);
    } else {
        mVertexAttrib0Vector[0] = PuntToFloat(x);
        mVertexAttrib0Vector[1] = PuntToFloat(y);
        mVertexAttrib0Vector[2] = PuntToFloat(z);
        mVertexAttrib0Vector[3] = PuntToFloat(w);
    }
}

void
WebGL2Context::VertexAttribI4uiv(GLuint index, size_t length, const GLuint* v)
{
    if (IsContextLost())
        return;

    if (index || gl->IsGLES()) {
        MakeContextCurrent();
        gl->fVertexAttribI4uiv(index, v);
    } else {
        mVertexAttrib0Vector[0] = PuntToFloat(v[0]);
        mVertexAttrib0Vector[1] = PuntToFloat(v[1]);
        mVertexAttrib0Vector[2] = PuntToFloat(v[2]);
        mVertexAttrib0Vector[3] = PuntToFloat(v[3]);
    }
}

void
WebGL2Context::VertexAttribI4uiv(GLuint index, const dom::Sequence<GLuint>& v)
{
    VertexAttribI4uiv(index, v.Length(), v.Elements());
}

// -------------------------------------------------------------------------
// Uniform Buffer Objects and Transform Feedback Buffers
// TODO(djg): Implemented in WebGLContext
/*
    void BindBufferBase(GLenum target, GLuint index, WebGLBuffer* buffer);
    void BindBufferRange(GLenum target, GLuint index, WebGLBuffer* buffer,
                         GLintptr offset, GLsizeiptr size);
*/

/* This doesn't belong here. It's part of state querying */
void
WebGL2Context::GetIndexedParameter(GLenum target, GLuint index,
                                   dom::Nullable<dom::OwningWebGLBufferOrLongLong>& retval)
{
    retval.SetNull();
    if (IsContextLost())
        return;

    GLint64 data = 0;

    MakeContextCurrent();

    switch (target) {
    case LOCAL_GL_TRANSFORM_FEEDBACK_BUFFER_BINDING:
        if (index >= mGLMaxTransformFeedbackSeparateAttribs)
            return ErrorInvalidValue("getIndexedParameter: index should be less than "
                                     "MAX_TRANSFORM_FEEDBACK_SEPARATE_ATTRIBS");

        retval.SetValue().SetAsWebGLBuffer() =
            mBoundTransformFeedbackBuffers[index].get();
        return;

    case LOCAL_GL_UNIFORM_BUFFER_BINDING:
        if (index >= mGLMaxUniformBufferBindings)
            return ErrorInvalidValue("getIndexedParameter: index should be than "
                                     "MAX_UNIFORM_BUFFER_BINDINGS");

        retval.SetValue().SetAsWebGLBuffer() = mBoundUniformBuffers[index].get();
        return;

    case LOCAL_GL_TRANSFORM_FEEDBACK_BUFFER_START:
    case LOCAL_GL_TRANSFORM_FEEDBACK_BUFFER_SIZE:
    case LOCAL_GL_UNIFORM_BUFFER_START:
    case LOCAL_GL_UNIFORM_BUFFER_SIZE:
        gl->fGetInteger64i_v(target, index, &data);
        retval.SetValue().SetAsLongLong() = data;
        return;
    }

    ErrorInvalidEnumInfo("getIndexedParameter: target", target);
}

void
WebGL2Context::GetUniformIndices(WebGLProgram* program,
                                 const dom::Sequence<nsString>& uniformNames,
                                 dom::Nullable< nsTArray<GLuint> >& retval)
{
    retval.SetNull();
    if (IsContextLost())
        return;

    if (!ValidateObject("getUniformIndices: program", program))
        return;

    if (!uniformNames.Length())
        return;

    GLuint progname = program->mGLName;
    size_t count = uniformNames.Length();
    nsTArray<GLuint>& arr = retval.SetValue();

    MakeContextCurrent();

    for (size_t n = 0; n < count; n++) {
        NS_LossyConvertUTF16toASCII name(uniformNames[n]);
        //        const GLchar* glname = name.get();
        const GLchar* glname = nullptr;
        name.BeginReading(glname);

        GLuint index = 0;
        gl->fGetUniformIndices(progname, 1, &glname, &index);
        arr.AppendElement(index);
    }
}

void
WebGL2Context::GetActiveUniforms(WebGLProgram* program,
                                 const dom::Sequence<GLuint>& uniformIndices,
                                 GLenum pname,
                                 dom::Nullable< nsTArray<GLint> >& retval)
{
    retval.SetNull();
    if (IsContextLost())
        return;

    if (!ValidateObject("getActiveUniforms: program", program))
        return;

    size_t count = uniformIndices.Length();
    if (!count)
        return;

    GLuint progname = program->mGLName;
    nsTArray<GLint>& arr = retval.SetValue();
    arr.SetLength(count);

    MakeContextCurrent();
    gl->fGetActiveUniformsiv(progname, count, uniformIndices.Elements(), pname,
                             arr.Elements());
}

GLuint
WebGL2Context::GetUniformBlockIndex(WebGLProgram* program,
                                    const nsAString& uniformBlockName)
{
    if (IsContextLost())
        return 0;

    if (!ValidateObject("getUniformBlockIndex: program", program))
        return 0;

    // Leave this unchecked for now.

    const NS_LossyConvertUTF16toASCII cname(uniformBlockName);

    GLuint progname = program->mGLName;

    MakeContextCurrent();
    return gl->fGetUniformBlockIndex(progname, cname.BeginReading());
}

static bool
GetUniformBlockActiveUniforms(gl::GLContext* gl, JSContext* cx,
                              WebGL2Context* owner, GLuint progname,
                              GLuint uniformBlockIndex,
                              JS::MutableHandleObject out_array)
{
    GLint length = 0;
    gl->fGetActiveUniformBlockiv(progname, uniformBlockIndex,
                                 LOCAL_GL_UNIFORM_BLOCK_ACTIVE_UNIFORMS, &length);
    JS::RootedObject obj(cx, Uint32Array::Create(cx, owner, length, nullptr));
    if (!obj)
        return false;

    Uint32Array result;
    DebugOnly<bool> inited = result.Init(obj);
    MOZ_ASSERT(inited);
    result.ComputeLengthAndData();
    gl->fGetActiveUniformBlockiv(progname, uniformBlockIndex,
                                 LOCAL_GL_UNIFORM_BLOCK_ACTIVE_UNIFORM_INDICES,
                                 (GLint*) result.Data());

    out_array.set(obj);
    return true;
}

void
WebGL2Context::GetActiveUniformBlockParameter(JSContext* cx, WebGLProgram* program,
                                              GLuint uniformBlockIndex, GLenum pname,
                                              Nullable<dom::OwningUnsignedLongOrUint32ArrayOrBoolean>& retval,
                                              ErrorResult& rv)
{
    retval.SetNull();
    if (IsContextLost())
        return;

    if (!ValidateObject("getActiveUniformBlockParameter: program", program))
        return;

    GLuint progname = program->mGLName;
    GLint param = 0;

    MakeContextCurrent();

    switch(pname) {
    case LOCAL_GL_UNIFORM_BLOCK_REFERENCED_BY_VERTEX_SHADER:
    case LOCAL_GL_UNIFORM_BLOCK_REFERENCED_BY_FRAGMENT_SHADER:
        gl->fGetActiveUniformBlockiv(progname, uniformBlockIndex, pname, &param);
        retval.SetValue().SetAsBoolean() = (param != 0);
        return;

    case LOCAL_GL_UNIFORM_BLOCK_BINDING:
    case LOCAL_GL_UNIFORM_BLOCK_DATA_SIZE:
    case LOCAL_GL_UNIFORM_BLOCK_NAME_LENGTH:
    case LOCAL_GL_UNIFORM_BLOCK_ACTIVE_UNIFORMS:
        gl->fGetActiveUniformBlockiv(progname, uniformBlockIndex, pname, &param);
        retval.SetValue().SetAsUnsignedLong() = param;
        return;

    case LOCAL_GL_UNIFORM_BLOCK_ACTIVE_UNIFORM_INDICES:
        JS::RootedObject array(cx);
        if (!GetUniformBlockActiveUniforms(gl, cx, this, progname, uniformBlockIndex,
                                           &array))
        {
            rv = NS_ERROR_OUT_OF_MEMORY;
            return;
        }

        DebugOnly<bool> inited = retval.SetValue().SetAsUint32Array().Init(array);
        MOZ_ASSERT(inited);

        return;
    }

    ErrorInvalidEnumInfo("getActiveUniformBlockParameter: parameter", pname);
}

#define WEBGL_MAX_UNIFORM_BLOCK_NAME_LENGTH 256

void
WebGL2Context::GetActiveUniformBlockName(WebGLProgram* program, GLuint uniformBlockIndex,
                                         nsAString& retval)
{
    if (IsContextLost())
        return;

    if (!ValidateObject("getActiveUniformBlockName: program", program))
        return;

    GLuint progname = program->mGLName;
    GLchar nameBuffer[WEBGL_MAX_UNIFORM_BLOCK_NAME_LENGTH];
    GLsizei length = 0;

    MakeContextCurrent();
    gl->fGetActiveUniformBlockName(progname, uniformBlockIndex,
                                   WEBGL_MAX_UNIFORM_BLOCK_NAME_LENGTH, &length,
                                   nameBuffer);
    retval.Assign(NS_ConvertASCIItoUTF16(nsDependentCString(nameBuffer)));
}

#undef WEBGL_MAX_UNIFORM_BLOCK_NAME_LENGTH

void
WebGL2Context::UniformBlockBinding(WebGLProgram* program, GLuint uniformBlockIndex,
                                   GLuint uniformBlockBinding)
{
    if (IsContextLost())
        return;

    if (!ValidateObject("uniformBlockBinding: program", program))
        return;

    GLuint progname = program->mGLName;

    MakeContextCurrent();
    gl->fUniformBlockBinding(progname, uniformBlockIndex, uniformBlockBinding);
}
