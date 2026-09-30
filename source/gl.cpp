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

#include <SDL.h>
#include "gl.h"

#ifdef GL_LOADER_NONE

bool loadGL()
{
    return true;
}

#else

#define DEFINE_GL_FUNCTION(type, name) type gl_##name = nullptr;
GL_FUNCTION_LIST(DEFINE_GL_FUNCTION)
#undef DEFINE_GL_FUNCTION

bool loadGL()
{
    bool ok = true;
#define LOAD_GL_FUNCTION(type, name) ok &= (gl_##name = (type)SDL_GL_GetProcAddress(#name)) != nullptr;
    GL_FUNCTION_LIST(LOAD_GL_FUNCTION)
#undef LOAD_GL_FUNCTION
    return ok;
}

#endif
