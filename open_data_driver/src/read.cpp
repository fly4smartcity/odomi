#include <read/read.h>



    

	static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
	{
		//return fwrite(contents, size, nmemb, stream);
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
	}

    Read::Read() {
        headers = NULL;
        url = "";
    }

    Read::~Read() {
    }

    std::string Read::read(string file) {
		string buffer ="";
					//to save byte received on a file
			/*FILE* ofile;
			//fopen_s(&ofile,"networkrankF", "w");
			ofile = fopen("networkrankF","w");
			
			if(!ofile) {
        cout<<"Errore nella creazione del file!";
        return "";
    }*/

		
		url = file;
        curl_slist_append(headers, "Accept: application/json");
        curl_slist_append(headers, "Content-Type: application/json");
        curl_slist_append(headers, "charset: utf-8");
        curl = curl_easy_init();
        //string completeUrl = url.append(file);
        if (curl) {
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            //curl_easy_setopt(curl, CURLOPT_URL, completeUrl.c_str());
			curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPGET, 1);
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
			
			//to save byte received on a string
			curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
			curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);

//			curl_easy_setopt(curl,CURLOPT_CONNECT_ONLY);
            res = curl_easy_perform(curl);
//			curl_easy_recv(curl,buffer,1024,dim_letti);


            //fclose(ofile);

            if (res != CURLE_OK) {
                cerr << "Error: " << curl_easy_strerror(res) << endl;
            }
            curl_easy_cleanup(curl);
            curl_global_cleanup();
        }

	    //cout << buffer;
		return buffer;
    }
