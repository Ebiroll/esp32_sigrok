
#include "cfgmap.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

using namespace std;
//******************************************************************************

void EscapeStr(const string & input, string & output)
{
    string::const_iterator ch;
    for(ch = input.begin(); ch != input.end(); ++ch)
    {
        if(isprint(*ch) && *ch != '\\')
        {
            output += *ch;
        }
        else
        {
            char bfr[16];
            snprintf(bfr, 16, "\\%04X", (int)*ch);
            output += bfr;
        }
    }
} // EscapeStr()

void UnescapeStr(const string & input, string & output)
{
    string::const_iterator ch;
    ch = input.begin();
    while(ch != input.end())
    {
        if(*ch == '\\')
        {
            ++ch;
            char bfr[16];
            int j = 0;
            while(j < 4 && ch != input.end())
                bfr[j++] = *(ch++);
            bfr[j] = '\0';
            output += (char)strtol(bfr, NULL, 16);
        }
        else
        {
            output += *(ch++);
        }
    }
} // UnescapeStr()

void ReadParams(const std::string & path, configmap_t & params)
{
    FILE * fin = fopen(path.c_str(), "rb");
    int ch = fgetc(fin);
    while(ch != EOF)
    {
        string key;
        while(ch != EOF && ch != ':') {
            key += (char)ch;
            ch = fgetc(fin);
        }
        
        if(ch == ':')
            ch = fgetc(fin);
        else
            break;
        
        string encValue;
        while(ch != EOF && ch != '\n') {
            encValue += (char)ch;
            ch = fgetc(fin);
        }
        string value;
        UnescapeStr(encValue, value);
        params[key] = value;
        
        if(ch == '\n')
            ch = fgetc(fin);
    }
    fclose(fin);
} // ReadParams()

void WriteParams(const std::string & path, const configmap_t & params)
{
    FILE * fout = fopen(path.c_str(), "wb");
    configmap_t::const_iterator pitr;
    for(pitr = params.begin(); pitr != params.end(); ++pitr)
    {
        string escaped;
        EscapeStr(pitr->second, escaped);
        // Don't try to merge these. c_str() uses a static buffer.
        fprintf(fout, "%s:", pitr->first.c_str());
        fprintf(fout, "%s\n", escaped.c_str());
    }
    fclose(fout);
} // WriteParams()

//******************************************************************************

// g++ -DTEST_CFGMAP cfgmap.cpp -o test_cfgmap
#ifdef TEST_CFGMAP

#include <list>
#include <vector>
#include <stdarg.h>


int testsFailed = 0;

void TestValue(const std::string & key, const std::string & value, const configmap_t & params)
{
    configmap_t::const_iterator pitr = params.find(key);
    if(pitr == params.end()) {
        cerr << "Test failed, key \"" << key << "\" not found in params" << endl;
        exit(EXIT_FAILURE);
    }
    if(pitr->second != value) {
        ++testsFailed;
        cerr << "Test failed, expected value \"" << value << "\", found \"" << pitr->second << "\"" << endl;
    }
}

void TestSplit(const char * s, char ch, size_t n, ...)
{
    vector<string> result;
    split(s, ch, result);
    
    if(result.size() != n) {
        cerr << "split() test failed: split(" << s << ")" << endl;
        cerr << "Incorrect size: " << result.size() << ", expected: " << n << endl;
        ++testsFailed;
        return;
    }
    
    vector<string>::iterator subs = result.begin();
    va_list va;
    va_start(va, n);
    for(int j = 0; j < n; ++j, ++subs)
    {
        char * expSubs = va_arg(va, char *);
        if(*subs != expSubs) {
            cerr << "split() test failed: split(" << s << ")" << endl;
            cerr << "Incorrect substring: \"" << *subs << "\", expected \"" << expSubs << "\"" << endl;
            ++testsFailed;
            break;
        }
    }
    va_end(va);
}

int main(int argc, char * argv[])
{
    configmap_t params;
    params["foo"] = "bar";
    params[""] = "keyless";
    params["valueless"] = "";
    params["Sernum"] = "5164:6135:FA";
    params["addr"] = "192.168.3.21";
    params["complexString"] = "\\1, 2, 3,\n\\4, 5, 6";
    WriteParams("test.cfg", params);
    
    configmap_t rparams;
    ReadParams("test.cfg", rparams);
    
    configmap_t::const_iterator pitr;
    for(pitr = params.begin(); pitr != params.end(); ++pitr)
        TestValue(pitr->first, pitr->second, rparams);
    
    
    TestSplit("a:b:c", ':', 3, "a", "b", "c");
    TestSplit(":a:b:c:", ':', 5, "", "a", "b", "c", "");
    TestSplit("abc:def:ghi", ':', 3, "abc", "def", "ghi");
    TestSplit(":abc:def:ghi:", ':', 5, "", "abc", "def", "ghi", "");
    TestSplit("", ':', 1, "");
    TestSplit(":", ':', 2, "", "");
    
    
    cerr << testsFailed << " tests failed" << endl;
    
    return EXIT_SUCCESS;
}
#endif

