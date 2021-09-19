#pragma once

// http://stackoverflow.com/questions/2270726/how-to-determine-the-size-of-an-array-of-strings-in-c
template <typename T, size_t N>
char (&static_sizeof_array( T(&)[N] ))[N]; // declared, not defined
#define SIZEOF_ARRAY( x ) sizeof(static_sizeof_array(x))
