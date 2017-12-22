
#ifndef CFGMAP_H
#define CFGMAP_H

#include <string>
#include <map>

//******************************************************************************

// A simple text-based key-value store.
//   Read and write key-value parameters in newline-terminated, colon-separated format.
// Each line starts with the key, which consists of all characters up to the colon, and
// the value consists of all characters from the colon up to the first newline.
//   All printable characters are written directly, with one exception: \ characters are
// escaped as \005C. All unprintable characters are similarly escaped, so arbitrary binary
// data can be stored or even hand-written.

typedef std::map<std::string, std::string> configmap_t;

inline bool Get(const configmap_t & cfg, const std::string & key, std::string & value)
{
    std::map<std::string, std::string>::const_iterator ent = cfg.find(key);
    if(ent != cfg.end()) {
        value = ent->second;
        return true;
    }
    return false;
}

// Collect all keys with given prefix. Use to emulate hierarchical structure, arrays, etc.
template<typename cont_t>
inline void KeysWithPrefix(const configmap_t & cfg, const std::string & prefix, cont_t & keys)
{
    std::map<std::string, std::string>::const_iterator ent;
    for(ent = cfg.begin(); ent != cfg.end(); ++ent)
        if(ent->first.substr(0, prefix.length()) == prefix)
            keys.push_back(ent->first);
}

// A variant of KeysWithPrefix(): rather than collecting the complete keys into a list or vector,
// creates another map associating each value with the remainder of the keys.
// That is, given:
//  foo.name:thing 1
//  foo.size:42
//  foo.color:green
//  bar.name:thing 2
//  bar.size:3.14159
//  bar.color:microwave
//
// KeysWithPrefix() with a prefix of "foo." will give:
//  name:thing 1
//  size:42
//  color:green
//
inline void EntriesWithPrefix(const configmap_t & cfg, const std::string & prefix, configmap_t & submap)
{
    std::map<std::string, std::string>::const_iterator ent;
    for(ent = cfg.begin(); ent != cfg.end(); ++ent)
        if(ent->first.substr(0, prefix.length()) == prefix)
            submap[ent->first.substr(prefix.length())] = ent->second;
}

// Split a string into substrings on a given delimeter
// Splitting an empty string yields an empty string.
// A delimiter at the start or end of the string produces an empty string.
// split("a:b:c", ':') -> "a", "b", "c"
// split("", ':') -> ""
// split(":", ':') -> "", ""
// split(":a:b:c:", ':') -> "", "a", "b", "c", ""
template<typename cont_t>
inline void split(const std::string & str, char ch, cont_t & result)
// inline void split(const std::string & str, char ch, std::vector<std::string> & result)
{
    if(str.length() == 0) {
        result.push_back("");
        return;
    }
    
    if(str.length() == 1)
    {
        if(str[0] != ch) {
            result.push_back(str);
        }
        else {
            result.push_back("");
            result.push_back("");
        }
        return;
    }
    
    std::string::const_iterator begin, end;
    begin = str.begin();
    end = begin;
    while(begin < str.end())
    {
        while(end < str.end() && *end != ch)
            ++end;
        
        result.push_back(std::string(begin, end));
        begin = end + 1;
        end = begin;
    }
    if(str[str.length() - 1] == ch)
        result.push_back("");
}

void ReadParams(const std::string & path, configmap_t & params);
void WriteParams(const std::string & path, const configmap_t & params);

//******************************************************************************
#endif // CFGMAP_H
