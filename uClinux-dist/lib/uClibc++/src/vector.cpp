/*	Copyright (C) 2004 Garrett A. Kajmowicz

	This file is part of the uClibc++ Library.

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#define __UCLIBCXX_COMPILE_VECTOR__ 1


#include <vector>

namespace std{


#ifdef __UCLIBCXX_EXPAND_VECTOR_BASIC__

#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT vector<char>::vector();
	template _UCXXEXPORT vector<char>::vector(size_type n, const char & value);

	template _UCXXEXPORT vector<char>::~vector();
	template _UCXXEXPORT vector<unsigned char>::~vector();

#endif //__UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT void vector<char>::reserve(size_type n);
	template _UCXXEXPORT void vector<unsigned char>::reserve(size_type n);
	template _UCXXEXPORT void vector<short int>::reserve(size_type n);
	template _UCXXEXPORT void vector<unsigned short int>::reserve(size_type n);
	template _UCXXEXPORT void vector<int>::reserve(size_type n);
	template _UCXXEXPORT void vector<unsigned int>::reserve(size_type n);
	template _UCXXEXPORT void vector<bool>::reserve(size_type n);

	template _UCXXEXPORT void vector<char>::resize(size_type sz, const char & c);
	template _UCXXEXPORT void vector<unsigned char>::resize(size_type sz, const unsigned char & c);
	template _UCXXEXPORT void vector<short int>::resize(size_type sz, const short & c);
	template _UCXXEXPORT void vector<unsigned short int>
		::resize(size_type sz, const unsigned short int & c);
	template _UCXXEXPORT void vector<int>::resize(size_type sz, const int & c);
	template _UCXXEXPORT void vector<unsigned int>::resize(size_type sz, const unsigned int & c);
	template _UCXXEXPORT void vector<bool>::resize(size_type sz, const bool & c);

#elif defined __UCLIBCXX_EXPAND_STRING_CHAR__


#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__
	template _UCXXEXPORT vector<char>::vector();
	template _UCXXEXPORT vector<char>::vector(size_type n, const char & value);
	template _UCXXEXPORT vector<char>::~vector();
#endif // __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT void vector<char>::reserve(size_type n);
	template _UCXXEXPORT void vector<char>::resize(size_type sz, const char & c);

#endif




}
