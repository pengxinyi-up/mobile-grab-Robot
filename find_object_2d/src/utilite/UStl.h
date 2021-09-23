// Taken from UtiLite library r185 [www.utilite.googlecode.com]

/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef USTL_H
#define USTL_H

#include <list>
#include <map>
#include <set>
#include <vector>
#include <string>
#include <algorithm>

/**
 * \file UStl.h
 * \brief Wrappers of STL for convenient functions.
 *
 * All functions you will find here are here
 * for the use of STL in a more convenient way.
 */


/**
 * Get unique keys from a std::multimap.
 * @param mm the multimap
 * @return the list which contains unique keys
 */
template<class K, class V>
inline std::list<K> uUniqueKeys(const std::multimap<K, V> & mm)
{
	std::list<K> l;
	typename std::list<K>::reverse_iterator lastValue;
	for(typename std::multimap<K, V>::const_iterator iter = mm.begin(); iter!=mm.end(); ++iter)
	{
		if(iter == mm.begin() || (iter != mm.begin() && *lastValue != iter->first))
		{
			l.push_back(iter->first);
			lastValue = l.rbegin();
		}
	}
	return l;
}

/**
 * Get all keys from a std::multimap.
 * @param mm the multimap
 * @return the list which contains all keys (may contains duplicated keys)
 */
template<class K, class V>
inline std::list<K> uKeys(const std::multimap<K, V> & mm)
{
	std::list<K> l;
	for(typename std::multimap<K, V>::const_iterator iter = mm.begin(); iter!=mm.end(); ++iter)
	{
		l.push_back(iter->first);
	}
	return l;
}

/**
 * Get all values from a std::multimap.
 * @param mm the multimap
 * @return the list which contains all values (contains values from duplicated keys)
 */
template<class K, class V>
inline std::list<V> uValues(const std::multimap<K, V> & mm)
{
	std::list<V> l;
	for(typename std::multimap<K, V>::const_iterator iter = mm.begin(); iter!=mm.end(); ++iter)
	{
		l.push_back(iter->second);
	}
	return l;
}

/**
 * Get values for a specified key from a std::multimap.
 * @param mm the multimap
 * @param key the key
 * @return the list which contains the values of the key
 */
template<class K, class V>
inline std::list<V> uValues(const std::multimap<K, V> & mm, const K & key)
{
	std::list<V> l;
	std::pair<typename std::multimap<K, V>::const_iterator, typename std::multimap<K, V>::const_iterator> range;
	range = mm.equal_range(key);
	for(typename std::multimap<K, V>::const_iterator iter = range.first; iter!=range.second; ++iter)
	{
		l.push_back(iter->second);
	}
	return l;
}

/**
 * Get all keys from a std::map.
 * @param m the map
 * @return the vector of keys
 */
template<class K, class V>
inline std::vector<K> uKeys(const std::map<K, V> & m)
{
	std::vector<K> v(m.size());
	int i=0;
	for(typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter)
	{
		v[i] = iter->first;
		++i;
	}
	return v;
}

/**
 * Get all keys from a std::map.
 * @param m the map
 * @return the list of keys
 */
template<class K, class V>
inline std::list<K> uKeysList(const std::map<K, V> & m)
{
	std::list<K> l;
	for(typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter)
	{
		l.push_back(iter->first);
	}
	return l;
}

/**
 * Get all keys from a std::map.
 * @param m the map
 * @return the set of keys
 */
template<class K, class V>
inline std::set<K> uKeysSet(const std::map<K, V> & m)
{
	std::set<K> s;
	int i=0;
	for(typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter)
	{
		s.insert(s.end(), iter->first);
		++i;
	}
	return s;
}

/**
 * Get all values from a std::map.
 * @param m the map
 * @return the vector of values
 */
template<class K, class V>
inline std::vector<V> uValues(const std::map<K, V> & m)
{
	std::vector<V> v(m.size());
	int i=0;
	for(typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter)
	{
		v[i] = iter->second;
		++i;
	}
	return v;
}

/**
 * Get all values from a std::map.
 * @param m the map
 * @return the list of values
 */
template<class K, class V>
inline std::list<V> uValuesList(const std::map<K, V> & m)
{
	std::list<V> l;
	for(typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter)
	{
		l.push_back(iter->second);
	}
	return l;
}

/**
 * Get the value of a specified key from a std::map.
 * @param m the map
 * @param key the key
 * @param defaultValue the default value used if the key is not found
 * @return the value
 */
template<class K, class V>
inline V uValue(const std::map<K, V> & m, const K & key, const V & defaultValue = V())
{
	V v = defaultValue;
	typename std::map<K, V>::const_iterator i = m.find(key);
	if(i != m.end())
	{
		v = i->second;
	}
	return v;
}

/**
 * Get the value of a specified key from a std::map. This will
 * remove the value from the map;
 * @param m the map
 * @param key the key
 * @param defaultValue the default value used if the key is not found
 * @return the value
 */
template<class K, class V>
inline V uTake(std::map<K, V> & m, const K & key, const V & defaultValue = V())
{
	V v;
	typename std::map<K, V>::iterator i = m.find(key);
	if(i != m.end())
	{
		v = i->second;
		m.erase(i);
	}
	else
	{
		v = defaultValue;
	}
	return v;
}

/**
 * Get the iterator at a specified position in a std::list. If the position
 * is out of range, the result is the end iterator of the list.
 * @param list the list
 * @param pos the index position in the list
 * @return the iterator at the specified index
 */
template<class V>
inline typename std::list<V>::iterator uIteratorAt(std::list<V> & list, const unsigned int & pos)
{
	typename std::list<V>::iterator iter = list.begin();
	for(unsigned int i = 0; i<pos && iter != list.end(); ++i )
	{
		++iter;
	}
	return iter;
}

/**
 * Get the iterator at a specified position in a std::list. If the position
 * is out of range, the result is the end iterator of the list.
 * @param list the list
 * @param pos the index position in the list
 * @return the iterator at the specified index
 */
template<class V>
inline typename std::list<V>::const_iterator uIteratorAt(const std::list<V> & list, const unsigned int & pos)
{
	typename std::list<V>::const_iterator iter = list.begin();
	for(unsigned int i = 0; i<pos && iter != list.end(); ++i )
	{
		++iter;
	}
	return iter;
}

/**
 * Get the iterator at a specified position in a std::vector. If the position
 * is out of range, the result is the end iterator of the vector.
 * @param v the vector
 * @param pos the index position in the vector
 * @return the iterator at the specified index
 */
template<class V>
inline typename std::vector<V>::iterator uIteratorAt(std::vector<V> & v, const unsigned int & pos)
{
	return v.begin() + pos;
}

/**
 * Get the value at a specified position in a std::list. If the position
 * is out of range, the result is undefined.
 * @param list the list
 * @param pos the index position in the list
 * @return the value at the specified index
 */
template<class V>
inline V & uValueAt(std::list<V> & list, const unsigned int & pos)
{
	typename std::list<V>::iterator iter = uIteratorAt(list, pos);
	return *iter;
}

/**
 * Get the value at a specified position in a std::list. If the position
 * is out of range, the result is undefined.
 * @param list the list
 * @param pos the index position in the list
 * @return the value at the specified index
 */
template<class V>
inline const V & uValueAt(const std::list<V> & list, const unsigned int & pos)
{
	typename std::list<V>::const_iterator iter = uIteratorAt(list, pos);
	return *iter;
}

/**
 * Check if the list contains the specified value.
 * @param list the list
 * @param value the value
 * @return true if the value is found in the list, otherwise false
 */
template<class V>
inline bool uContains(const std::list<V> & list, const V & value)
{
	return std::find(list.begin(), list.end(), value) != list.end();
}

/**
 * Check if the map contains the specified key.
 * @param map the map
 * @param key the key
 * @return true if the value is found in the map, otherwise false
 */
template<class K, class V>
inline bool uContains(const std::map<K, V> & map, const K & key)
{
	return map.find(key) != map.end();
}

/**
 * Check if the multimap contains the specified key.
 * @param map the map
 * @param key the key
 * @return true if the value is found in the map, otherwise false
 */
template<class K, class V>
inline bool uContains(const std::multimap<K, V> & map, const K & key)
{
	return map.find(key) != map.end();
}

/**
 * Insert an item in the map. Contrary to the insert in the STL,
 * if the key already exists, the value will be replaced by the new one.
 */
template<class K, class V>
inline void uInsert(std::map<K, V> & map, const std::pair<K, V> & pair)
{
	std::pair<typename std::map<K, V>::iterator, bool> inserted = map.insert(pair);
	if(inserted.second == false)
	{
		inserted.first->second = pair.second;
	}
}

/**
 * Convert a std::list to a std::vector.
 * @param list the list
 * @return the vector
 */
template<class V>
inline std::vector<V> uListToVector(const std::list<V> & list)
{
	return std::vector<V>(list.begin(), list.end());
}

/**
 * Convert a std::vector to a std::list.
 * @param v the vector
 * @return the list
 */
template<class V>
inline std::list<V> uVectorToList(const std::vector<V> & v)
{
	return std::list<V>(v.begin(), v.end());
}

/**
 * Append a list to another list.
 * @param list the list on which the other list will be appended
 * @param newItems the list of items to be appended
 */
template<class V>
inline void uAppend(std::list<V> & list, const std::list<V> & newItems)
{
	list.insert(list.end(), newItems.begin(), newItems.end());
}

/**
 * Get the index in the list of the specified value. S negative index is returned
 * if the value is not found.
 * @param list the list
 * @param value the value
 * @return the index of the value in the list
 */
template<class V>
inline int uIndexOf(const std::vector<V> & list, const V & value)
{
	int index=-1;
	int i=0;
	for(typename std::vector<V>::const_iterator iter = list.begin(); iter!=list.end(); ++iter)
	{
		if(*iter == value)
		{
			index = i;
			break;
		}
		++i;
	}
	return index;
}

/**
 * Split a string into multiple string around the specified separator.
 * Example:
 * @code
 * 		std::list<std::string> v = split("Hello the world!", ' ');
 * @endcode
 * The list v will contain {"Hello", "the", "world!"}
 * @param str the string
 * @param separator the separator character
 * @return the list of strings
 */
inline std::list<std::string> uSplit(const std::string & str, char separator = ' ')
{
	std::list<std::string> v;
	std::string buf;
	for(unsigned int i=0; i<str.size(); ++i)
	{
		if(str[i] != separator)
		{
			buf += str[i];
		}
		else if(buf.size())
		{
			v.push_back(buf);
			buf = "";
		}
	}
	if(buf.size())
	{
		v.push_back(buf);
	}
	return v;
}

/**
 * Check if a character is a digit.
 * @param c the character
 * @return if the character is a digit (if c >= '0' && c <= '9')
 */
inline bool uIsDigit(const char c)
{
	return c >= '0' && c <= '9';
}

/**
 * Split a string into number and character strings.
 * Example:
 * @code
 * 		std::list<std::string> v = uSplit("Hello 03 my 65 world!");
 * @endcode
 * The list v will contain {"Hello ", "03", " my ", "65", " world!"}
 * @param str the string
 * @return the list of strings
 */
inline std::list<std::string> uSplitNumChar(const std::string & str)
{
	std::list<std::string> list;
	std::string buf;
	bool num = false;
	for(unsigned int i=0; i<str.size(); ++i)
	{
		if(uIsDigit(str[i]))
		{
			if(!num && buf.size())
			{
				list.push_back(buf);
				buf.clear();
			}
			buf += str[i];
			num = true;
		}
		else
		{
			if(num)
			{
				list.push_back(buf);
				buf.clear();
			}
			buf += str[i];
			num = false;
		}
	}
	if(buf.size())
	{
		list.push_back(buf);
	}
	return list;
}

/**
 * Compare two alphanumeric strings. Useful to sort filenames (human-like sorting).
 * Example:
 * @code
 * 	std::string a = "Image9.jpg";
 * 	std::string b = "Image10.jpg";
 * 	int r = uStrNumCmp(a, b); // r returns -1 (a is smaller than b). In contrast, std::strcmp(a, b) would return 1.
 * @endcode
 * @param a the first string
 * @param b the second string
 * @return -1 if a<b, 0 if a=b and 1 if a>b
 */
inline int uStrNumCmp(const std::string & a, const std::string & b)
{
	std::vector<std::string> listA;
	std::vector<std::string> listB;

	listA = uListToVector(uSplitNumChar(a));
	listB = uListToVector(uSplitNumChar(b));

	unsigned int i;
	int result = 0;
	for(i=0; i<listA.size() && i<listB.size(); ++i)
	{
		if(uIsDigit(listA[i].at(0)) && uIsDigit(listB[i].at(0)))
		{
			//padding if zeros at the beginning
			if(listA[i].at(0) == '0' && listB[i].size() < listA[i].size())
			{
				while(listB[i].size() < listA[i].size())
				{
					listB[i] += '0';
				}
			}
			else if(listB[i].at(0) == '0' && listA[i].size() < listB[i].size())
			{
				while(listA[i].size() < listB[i].size())
				{
					listA[i] += '0';
				}
			}

			if(listB[i].size() < listA[i].size())
			{
				result = 1;
			}
			else if(listB[i].size() > listA[i].size())
			{
				result = -1;
			}
			else
			{
				result = listA[i].compare(listB[i]);
			}
		}
		else if(uIsDigit(listA[i].at(0)))
		{
			result = -1;
		}
		else if(uIsDigit(listB[i].at(0)))
		{
			result = 1;
		}
		else
		{
			result = listA[i].compare(listB[i]);
		}

		if(result != 0)
		{
			break;
		}
	}

	return result;
}

#endif /* USTL_H */
