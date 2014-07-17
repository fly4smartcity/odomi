

    #ifndef READ_H
    #define    READ_H

    #include <iostream>
	#include "curl/curl.h"

    using namespace std;

    class Read {
    public:
        Read();
        virtual ~Read();
        string read(string file);
    private:
        string url;
        CURL *curl;
        CURLcode res;
        struct curl_slist *headers;
    };

    #endif
