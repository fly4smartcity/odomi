#include <iostream>
#include <sstream>
#include <string>
#include <opendata/Trasporti.h>
#include <csv/csv_parser.hpp>
#include <zzip/lib.h>
#include <mysql/mysql.h>
#include "open_data_msg/Data.h"
#include "open_data_msg/Open_data.h"

using namespace std;

FILE *f;


	Trasporti::Trasporti(){}	

        Trasporti::~Trasporti(){}
	
	void Trasporti::initialize()
	{
	const int num_files=3;
	const int num_archives=2;
	string file_zip_name[num_archives];// = {"torino_it.zip","sfm_torino_it.zip"};
	string filename[num_files] = {"routes.txt","trips.txt","shapes.txt"};
	 ros::NodeHandle n;

	if(( !ros::param::get("/train_filename", file_zip_name[0])) || (!ros::param::get("/tram_filename", file_zip_name[1])) )
	{
		ROS_INFO("Errore: rosparam non corretti! Eseguire prima il comando \"rosparam load open_data.yaml\" ");
		exit(1);
	}
	

	{
		
	}


				if( downloadZip("http://opendata.5t.torino.it/gtfs/torino_it.zip",file_zip_name[1]) && downloadZip("http://opendata.5t.torino.it/gtfs/sfm_torino_it.zip",file_zip_name[0]) )
	{		
		    
			createDb();
		for(int i=0;i<num_archives;i++)
				extractZip(filename,num_files,file_zip_name[i]);                	       	             

	}

	}

	bool Trasporti::downloadZip(string url,string filename)
	{				
		Read rd;
		ROS_INFO("Scaricamento %s in corso...",filename.c_str());
		string s = rd.read(url);
		
		if(s == "")
		{
			ROS_INFO("File %s non e' stato scaricato. Verificare la propria connessione ad Internet.",filename.c_str());  
			return false;
		}
		
		
		ofstream of(filename.c_str());
		of << s;
		ROS_INFO("File %s scaricato completamente.",filename.c_str());
		return true;



	}

        void Trasporti::extractZip(string filenames[], int n , string archive)
        {
        	const int buflen = 1024;
        	char buffer[buflen]="";
        	int letti;
        	 //Open the ZIP archive TORINO_IT
       				ZZIP_DIR* dir = zzip_dir_open(archive.c_str(),0);

        				 if (dir)
        				 {
        					 for(int i=0;i<n;i++)
        					 {
        						 ZZIP_FILE* fp = zzip_file_open(dir,filenames[i].c_str(),0);
        						 if (fp)
								   {
        							 if(i==0)
        								 routesToDb(fp);
        							 else if(i==1)
        								 tripsToDb(fp);
        							 else if(i==2)
        								 shapesToDb(fp);

								   }
        						 zzip_file_close(fp);

						     }


        				 }
        				 zzip_dir_close(dir);

        }





  void Trasporti::createDb()
  {
	  MYSQL *conn,*c1;
	             MYSQL_RES *res;
	             MYSQL_ROW row;
	
//get database parameters
 ros::NodeHandle n;
string server,user,password,database;
				ros::param::get("/db_server", server);
				ros::param::get("/db_user", user);
				ros::param::get("/db_psw", password);
				ros::param::get("/db_name", database);


	             conn =  mysql_init(NULL);
	     	     c1 =mysql_init(NULL);

	             /* Connect to database */
		if (!mysql_real_connect(c1, (const char*)server.c_str(),(const char*)user.c_str(),(const char*)password.c_str(), NULL, 0, NULL, 0))
		{
			fprintf(stderr, "%s\n", mysql_error(c1));
								mysql_close(c1);
								exit(1);

		}

			if(mysql_query(c1,"DROP DATABASE IF EXISTS trasporti"))
			{
			fprintf(stderr, "%s\n", mysql_error(c1));
			mysql_close(c1);
			exit(1);	
			}
					 if (mysql_query(c1, "CREATE DATABASE trasporti"))
							{
								fprintf(stderr, "%s\n", mysql_error(c1));
								mysql_close(c1);
								exit(1);
							}
	ROS_INFO("DB %s creato con successo!",database.c_str());
		mysql_close(c1);	

						if (!mysql_real_connect(conn, (const char*)server.c_str(),(const char*)user.c_str(),(const char*)password.c_str(), (const char*)database.c_str(), 0, NULL, 0))
				{
					fprintf(stderr, "%s\n", mysql_error(conn));
										mysql_close(conn);
										exit(1);

				}

	             //creo le tabelle
				if(mysql_query(conn, "DROP TABLE IF EXISTS shapes"))
					    	    {
					    	        fprintf(stderr, "%s\n", mysql_error(conn));
					    	        mysql_close(conn);
					    	        exit(1);
					    	    }
	ROS_INFO("tabella shapes eliminata");

				if(mysql_query(conn, "DROP TABLE IF EXISTS trips"))
					    	    {
					    	        fprintf(stderr, "%s\n", mysql_error(conn));
					    	        mysql_close(conn);
					    	        exit(1);
					    	    }
	ROS_INFO("tabella trips eliminata");
	            if(mysql_query(conn, "DROP TABLE IF EXISTS routes"))
	    	    {
	    	        fprintf(stderr, "%s\n", mysql_error(conn));
	    	        mysql_close(conn);
	    	        exit(1);
	    	    }
	ROS_INFO("tabella routes eliminata");




	  if (mysql_query(conn, "CREATE TABLE routes(route_id VARCHAR(30) PRIMARY KEY,route_type INT)"))
	    {
	        fprintf(stderr, "%s\n", mysql_error(conn));
	        mysql_close(conn);
	        exit(1);
	    }
	ROS_INFO("tabella routes creato con successo!");
	  if (mysql_query(conn, "CREATE TABLE trips(route_id VARCHAR(30), shape_id VARCHAR(50) PRIMARY KEY ,FOREIGN KEY (route_id) REFERENCES routes(route_id))"))
	  	    {
	  	        fprintf(stderr, "%s\n", mysql_error(conn));
	  	        mysql_close(conn);
	  	        exit(1);
	  	    }
	ROS_INFO("tabella trips creato con successo!");

	  	  if (mysql_query(conn, "CREATE TABLE shapes(shape_id VARCHAR(50),shape_pt_lat1 float,shape_pt_long1 float,shape_pt_lat2 float,shape_pt_long2 float,KEY s (shape_id),FOREIGN KEY (shape_id) REFERENCES trips(shape_id))"))
	  	  	  	    {
	  	  	  	        fprintf(stderr, "%s\n", mysql_error(conn));
	  	  	  	        mysql_close(conn);
	  	  	  	        exit(1);
	  	  	  	    }
	ROS_INFO("tabella shapes creato con successo!");


	  	mysql_close(conn);

	ROS_INFO("Database creato corretamente");
  }



 open_data_msg::Data Trasporti::getShapesFromBoundingBox(float x1, float y1, float x2, float y2)
 {
	 MYSQL *conn;
	            MYSQL_RES *res;
	            MYSQL_ROW row;
	            conn = mysql_init(NULL);

		    open_data_msg::Data vettore;
		    open_data_msg::Open_data op;
		    bool ok =false;				
		    int numAttr =1; //unico attributo

       		    string label = "linee_aeree";
		    string key = "altezza_max";
		    string value = "6.0";

		//get database parameters
		string server,user,password,database;
		ros::param::get("/db_server", server);
		ros::param::get("/db_user", user);
		ros::param::get("/db_psw", password);
		ros::param::get("/db_name", database);

	 /* Connect to database */
	            if (!mysql_real_connect(conn, (const char*)server.c_str(),(const char*)user.c_str(),(const char*)password.c_str(), (const char*)database.c_str(), 0, NULL, 0)){
	               fprintf(stderr, "%s\n", mysql_error(conn));
	               exit(1);
	            }

	            if(x1 > x2)
	            {
	            	float tmp = x1;
	            	x1 = x2;
	            	x2 = tmp;
	            }

	            if(y1 > y2)
	            {
	            	float tmp = y1;
	            	y1 = y2;
	            	y2 = tmp;
	            }


	            stringstream sstm;
	            sstm << "SELECT shape_pt_lat1,shape_pt_long1,shape_pt_lat2,shape_pt_long2 FROM shapes ";
	            sstm << "WHERE ((shape_pt_long1 BETWEEN " << x1 <<"  AND " << x2 << ") ";
	            sstm << "OR (shape_pt_long2 BETWEEN " << x1 <<"  AND " << x2 << ")) ";
	            sstm << "AND ((shape_pt_lat1 BETWEEN " << y1 <<" AND " << y2 << ") ";
	            sstm << "OR (shape_pt_lat2 BETWEEN " << y1 <<" AND " << y2 << "))";

	            string s = sstm.str();
	            if (mysql_query(conn, s.c_str())) {
	                          fprintf(stderr, "%s\n", mysql_error(conn));
	                          exit(1);
	                       }

	            MYSQL_RES *result = mysql_store_result(conn);

	             if (result == NULL)
	             {
	            	 fprintf(stderr, "%s\n", mysql_error(conn));
			 exit(1);
	             }

	             int num_fields = mysql_num_fields(result);

	             MYSQL_ROW coord;
			
		     int featureNum=0;

	             while ((coord = mysql_fetch_row(result)))
	             {
			if(!ok)
			   ok=true;

			vettore.data.resize(featureNum+1);
			vettore.data[featureNum].type = op.TYPE_STATIC;
	            	 vettore.data[featureNum].label.assign(label);
         	   	  vettore.data[featureNum].attributes.resize(numAttr);
			 vettore.data[featureNum].attributes[0].key.assign(key);
			 vettore.data[featureNum].attributes[0].value.assign(value);
			vettore.data[featureNum].area.points.resize(2);
			vettore.data[featureNum].area.points[0].y = atof(coord[0]);
			vettore.data[featureNum].area.points[0].x = atof(coord[1]);
			vettore.data[featureNum].area.points[0].z = 0.0;
			vettore.data[featureNum].area.points[1].y = atof(coord[2]);
			vettore.data[featureNum].area.points[1].x = atof(coord[3]);
			vettore.data[featureNum].area.points[0].z = 0.0;

			featureNum++;
	             }

	             mysql_free_result(result);
	             mysql_close(conn);

	if(ok)
	{	
	     vettore.status.status_code = 1;
	     vettore.status.reason = "OK";
	}
	else
	{
			  vettore.status.status_code = -1;
			  vettore.status.reason = "Nessun Risultato";

	}

	return vettore;
 }

void Trasporti::tripsToDb(ZZIP_FILE* file_zip)
 {
	ROS_INFO("leggo trips");
			     MYSQL *conn;
	       	             MYSQL_RES *res;

	       	             conn = mysql_init(NULL);
string server,user,password,database;
ros::param::get("/db_server", server);
				ros::param::get("/db_user", user);
				ros::param::get("/db_psw", password);
				ros::param::get("/db_name", database);

	       	             /* Connect to database */
	       	             if (!mysql_real_connect(conn, (const char*)server.c_str(),(const char*)user.c_str(),(const char*)password.c_str(), (const char*)database.c_str(), 0, NULL, 0)){
	       	                fprintf(stderr, "%s\n", mysql_error(conn));
	       	                exit(1);
	       	             }

	     		const char * filename = "trips.txt";

	             const char field_terminator = ',';
	             const char line_terminator  = '\n';
	             const char enclosure_char   = '"';


	             csv_parser file_parser;

	             file_parser.set_skip_lines(0);

	             //file_parser.init(filename);
	             file_parser.init_zip(file_zip);

	     		file_parser.set_enclosed_char(enclosure_char, file_parser.ENCLOSURE_OPTIONAL);

	     		file_parser.set_field_term_char(field_terminator);
	             file_parser.set_line_term_char(line_terminator);

	             /*Controllo in quale colonna è presente lo shape_id */
	                         int indice=-1;

	                         csv_row row1 = file_parser.get_row_zip();
				int num_fields = row1.size();

        	                        for(int i=0; i< num_fields;i++)
	                         {
	                         	if(strcmp(row1[i].c_str(),"shape_id")==0)
	                         	{
	                         		indice = i;
	                         		break;
	                         	}

	                         }
	                          
	                         //indice=6;

	                        if(indice==-1)//campo shape_id non trovato!!!
	                        {
	                        	printf("campo shape_id non trovato!!!");
	                        	exit(1);
	                        }


	                        /* */

	              string prec_shape_id="";

		     csv_row row;
		    bool end=false;

	             while(!end)
	             {
	            	 row  = file_parser.get_row_zip();

		    	 if(row.size() < num_fields || row.empty())
	            	    end=true;

                	 else
	            	{
			        if(prec_shape_id == row[indice])
					continue;

				else
					prec_shape_id = row[indice];


			        stringstream sstm1;
        	                sstm1 << "SELECT COUNT(*) FROM routes WHERE route_id='" <<row[0].c_str()<<"'";
        	                string s1 = sstm1.str();

			        if (mysql_query(conn, s1.c_str()))
               	   	   	   {
		       	  	        fprintf(stderr, "%s\n", mysql_error(conn));
		       	  	        mysql_close(conn);
		       	  	        exit(1);
               	   	   	   }

			        MYSQL_RES *result = mysql_store_result(conn);
			        MYSQL_ROW coord = mysql_fetch_row(result);
			        char* c = coord[0];

					if(atoi(c) > 0)
					{

						printf("shape_id= %s\n",row[indice].c_str());
					stringstream sstm;
					//sstm << "INSERT INTO trips(route_id,shape_id) VALUES ('" << row[0].c_str() << "','" << row[indice].c_str() << "')";
					sstm << "INSERT INTO trips (route_id,shape_id) ";
					sstm << "SELECT * FROM (SELECT '" << row[0].c_str() << "','" << row[indice].c_str() << "') AS tmp ";
					sstm <<"WHERE NOT EXISTS (SELECT shape_id FROM trips WHERE shape_id = '" << row[indice].c_str() << "') LIMIT 1;";
					string s = sstm.str();
					//inserisco nel db
								   if (mysql_query(conn, s.c_str()))
											   {
												fprintf(stderr, "%s\n", mysql_error(conn));
												mysql_close(conn);
												exit(1);
											   }
					}
	        	}//chiude else di if(row.size()<num_fields)
	             }//chiude while(!end)
				  mysql_close(conn);

		ROS_INFO("Trips letti\n");


 }
                 void Trasporti::shapesToDb(ZZIP_FILE* file_zip)
                 {
	ROS_INFO("leggo shapes");
                 	     MYSQL *conn;
	 	             MYSQL_RES *res;
	 	             conn = mysql_init(NULL);

			//get database parameters
			string server,user,password,database;
			ros::param::get("/db_server", server);
			ros::param::get("/db_user", user);
			ros::param::get("/db_psw", password);
			ros::param::get("/db_name", database);


	 	             /* Connect to database */
	 	             if (!mysql_real_connect(conn, (const char*)server.c_str(),(const char*)user.c_str(),(const char*)password.c_str(), (const char*)database.c_str(), 0, NULL, 0)){
	 	                fprintf(stderr, "%s\n", mysql_error(conn));
	 	                exit(1);
	 	             }

               		const char * filename = "shapes.txt";

                       const char field_terminator = ',';
                       const char line_terminator  = '\n';
                       const char enclosure_char   = '"';
                       bool flag = false;

                       csv_parser file_parser;

                       file_parser.set_skip_lines(0);

               		//file_parser.init(filename);
                       file_parser.init_zip(file_zip);

               		file_parser.set_enclosed_char(enclosure_char, file_parser.ENCLOSURE_OPTIONAL);

               		file_parser.set_field_term_char(field_terminator);
                       file_parser.set_line_term_char(line_terminator);

                        csv_row row1 = file_parser.get_row_zip();
			int num_fields = row1.size();


                        string prec_shape_id="";
                        string lat_prec ="";
                        string long_prec="";

		     csv_row row;
	             bool end = false;

	             while(!end)
	             {
	            	 row  = file_parser.get_row_zip();

		    	 if(row.size() < num_fields || row.empty())
	            	    end=true;

                	 else
	            	{
				  if(prec_shape_id == row[0] && !flag)
					  continue;
				  
				  else if(prec_shape_id != row[0])
					flag=false;

		                  if(!flag && prec_shape_id != row[0].c_str())
		                  {

                          	 	stringstream sstm1;
	
					sstm1 << "SELECT COUNT(*) FROM trips WHERE shape_id='" <<row[0].c_str() <<"'";
					string s1 = sstm1.str();

					if (mysql_query(conn, s1.c_str()))
					   {
						fprintf(stderr, "%s\n", mysql_error(conn));
						mysql_close(conn);
						exit(1);
					   }

					MYSQL_RES *result = mysql_store_result(conn);
					MYSQL_ROW coord = mysql_fetch_row(result);
					char* c = coord[0];

					if(atoi(c) == 0)
						flag=false;
					else
					{
					printf("shape_id= %s\n",row[0].c_str());
					flag =true;
					lat_prec = row[1];
					long_prec = row[2];
					}
          			   }

		                  else if(atoi(row[3].c_str()) > 0 && flag)
		                  {
			stringstream sstm;
			sstm << "INSERT INTO shapes(shape_id,shape_pt_lat1,shape_pt_long1,shape_pt_lat2,shape_pt_long2) VALUES('";
			sstm << row[0].c_str() <<"'," << lat_prec << "," << long_prec << "," << row[1].c_str()<<"," << row[2].c_str() << ")";
				  	string s = sstm.str();
					//inserisco nel db
					 if (mysql_query(conn, s.c_str()))
								   {
									fprintf(stderr, "%s\n", mysql_error(conn));
									mysql_close(conn);
									exit(1);
								   }
					lat_prec = row[1];
					long_prec = row[2];
				  }

                         prec_shape_id = row[0];
                         }//chiude else di if(row.size()<num_fields)

		}//chiude while(!end)

                       mysql_close(conn);

		ROS_INFO("Shapes letti\n");


                 }

                 void Trasporti::routesToDb(ZZIP_FILE* file_zip)
                 {
		ROS_INFO("leggo routes!");
                	 MYSQL *conn;
                	       	             MYSQL_RES *res;
                	       	             conn = mysql_init(NULL);
	
				string server,user,password,database;

				//get database parameters
				ros::param::get("/db_server", server);
				ros::param::get("/db_user", user);
				ros::param::get("/db_psw", password);
				ros::param::get("/db_name", database);

                	       	             /* Connect to database */
                	       	             if (!mysql_real_connect(conn, (const char*)server.c_str(),(const char*)user.c_str(),(const char*)password.c_str(), (const char*)database.c_str(), 0, NULL, 0)){
                	       	                fprintf(stderr, "%s\n", mysql_error(conn));
                	       	                exit(1);
                	       	             }


                	     		const char * filename = "routes.txt";

                	             const char field_terminator = ',';
                	             const char line_terminator  = '\n';
                	             const char enclosure_char   = '"';
                	             int route_type_tram = 0;
                	             int route_type_train = 2;

                	             csv_parser file_parser;

                	             file_parser.set_skip_lines(0);

                	             //file_parser.init(filename);
                	             file_parser.init_zip(file_zip);


              	     		file_parser.set_enclosed_char(enclosure_char, file_parser.ENCLOSURE_OPTIONAL);
                	     		file_parser.set_field_term_char(field_terminator);
                	             file_parser.set_line_term_char(line_terminator);

                	             /*Controllo in quale colonna è presente il route_type */
                	                         int indice=-1;
                	                         csv_row row1 = file_parser.get_row_zip();
						int num_fields = row1.size();


                	                        for(int i=0; i< num_fields;i++)
                	                         {
                	                         	if(strcmp(row1[i].c_str(),"route_type")==0)
                	                         	{
                	                         		indice = i;
                	                         		break;
                	                         	}

                	                         }
                	                        //indice=5;
                	                        if(indice==-1)//campo route_type non trovato!!!
                	                        {
                	                        	printf("campo route_type non trovato!!!");
                	                        	exit(1);
                	                        }


                	                        /* */



				     csv_row row;
				    bool end=false;

                	             while(!end)
			             {

	        		    	 row  = file_parser.get_row_zip();

				    	 if(row.size() < num_fields || row.empty())
				    	    end=true;

					else
					{
                	                stringstream sstm;
                	                //printf("route_type: %s\n",row[indice].c_str());
                	                int type = atoi(row[indice].c_str());

						if(type == route_type_tram || type == route_type_train)
						{
							printf("route_id= %s\n",row[0].c_str());
						sstm << "INSERT INTO routes(route_id,route_type) VALUES('" << row[0].c_str() << "'," << row[indice].c_str() << ")";
						string s = sstm.str();

									   //inserisco nel db
									   if (mysql_query(conn, s.c_str()))
									   {
										fprintf(stderr, "%s\n", mysql_error(conn));
										mysql_close(conn);
										exit(1);
									   }
					    	}
					}//chiude else dell'if(row.size()<num_fields)
                	             }//chiude while(!end)
                 mysql_close(conn);
		
		ROS_INFO("Routes letti\n");

                 }

