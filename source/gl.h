/*
 * Copyright (c) 2026 Chris Giles
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Chris Giles makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */

#pragma once

// Minimal OpenGL header. The renderer targets the common subset of OpenGL 3.3 core and
// OpenGL ES 3.0 (WebGL2). On the web and on macOS every entry point is available directly,
// elsewhere the few functions newer than OpenGL 1.1 are loaded at startup through SDL.

#if defined(__EMSCRIPTEN__)
#include <GLES3/gl3.h>
#define GL_LOADER_NONE
#elif defined(__APPLE__)
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>
#define GL_LOADER_NONE
#else
#include <SDL_opengl.h>

#define GL_FUNCTION_LIST(X)                                        \
    X(PFNGLACTIVETEXTUREPROC, glActiveTexture)                     \
    X(PFNGLATTACHSHADERPROC, glAttachShader)                       \
    X(PFNGLBINDBUFFERPROC, glBindBuffer)                           \
    X(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer)                 \
    X(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray)                 \
    X(PFNGLBUFFERDATAPROC, glBufferData)                           \
    X(PFNGLCHECKFRAMEBUFFERSTATUSPROC, glCheckFramebufferStatus)   \
    X(PFNGLCOMPILESHADERPROC, glCompileShader)                     \
    X(PFNGLCREATEPROGRAMPROC, glCreateProgram)                     \
    X(PFNGLCREATESHADERPROC, glCreateShader)                       \
    X(PFNGLDELETESHADERPROC, glDeleteShader)                       \
    X(PFNGLDRAWBUFFERSPROC, glDrawBuffers)                         \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray) \
    X(PFNGLFRAMEBUFFERTEXTURE2DPROC, glFramebufferTexture2D)       \
    X(PFNGLGENBUFFERSPROC, glGenBuffers)                           \
    X(PFNGLGENFRAMEBUFFERSPROC, glGenFramebuffers)                 \
    X(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays)                 \
    X(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog)             \
    X(PFNGLGETPROGRAMIVPROC, glGetProgramiv)                       \
    X(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog)               \
    X(PFNGLGETSHADERIVPROC, glGetShaderiv)                         \
    X(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation)           \
    X(PFNGLLINKPROGRAMPROC, glLinkProgram)                         \
    X(PFNGLSHADERSOURCEPROC, glShaderSource)                       \
    X(PFNGLUNIFORM1FPROC, glUniform1f)                             \
    X(PFNGLUNIFORM1IPROC, glUniform1i)                             \
    X(PFNGLUNIFORM2FPROC, glUniform2f)                             \
    X(PFNGLUNIFORM3FPROC, glUniform3f)                             \
    X(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv)               \
    X(PFNGLUSEPROGRAMPROC, glUseProgram)                           \
    X(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer)

// Route calls through our own pointers (named gl_*) so they never clash with prototypes
// declared by the system headers.
#define DECLARE_GL_FUNCTION(type, name) extern type gl_##name;
GL_FUNCTION_LIST(DECLARE_GL_FUNCTION)
#undef DECLARE_GL_FUNCTION

#define glActiveTexture gl_glActiveTexture
#define glAttachShader gl_glAttachShader
#define glBindBuffer gl_glBindBuffer
#define glBindFramebuffer gl_glBindFramebuffer
#define glBindVertexArray gl_glBindVertexArray
#define glBufferData gl_glBufferData
#define glCheckFramebufferStatus gl_glCheckFramebufferStatus
#define glCompileShader gl_glCompileShader
#define glCreateProgram gl_glCreateProgram
#define glCreateShader gl_glCreateShader
#define glDeleteShader gl_glDeleteShader
#define glDrawBuffers gl_glDrawBuffers
#define glEnableVertexAttribArray gl_glEnableVertexAttribArray
#define glFramebufferTexture2D gl_glFramebufferTexture2D
#define glGenBuffers gl_glGenBuffers
#define glGenFramebuffers gl_glGenFramebuffers
#define glGenVertexArrays gl_glGenVertexArrays
#define glGetProgramInfoLog gl_glGetProgramInfoLog
#define glGetProgramiv gl_glGetProgramiv
#define glGetShaderInfoLog gl_glGetShaderInfoLog
#define glGetShaderiv gl_glGetShaderiv
#define glGetUniformLocation gl_glGetUniformLocation
#define glLinkProgram gl_glLinkProgram
#define glShaderSource gl_glShaderSource
#define glUniform1f gl_glUniform1f
#define glUniform1i gl_glUniform1i
#define glUniform2f gl_glUniform2f
#define glUniform3f gl_glUniform3f
#define glUniformMatrix4fv gl_glUniformMatrix4fv
#define glUseProgram gl_glUseProgram
#define glVertexAttribPointer gl_glVertexAttribPointer
#endif

// Loads the OpenGL entry points (no-op where they are linked directly). Returns false on failure.
bool loadGL();
