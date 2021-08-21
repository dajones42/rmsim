/*
Copyright © 2021 Doug Jones

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include "rmsim.h"
#include "rmssocket.h"
#include "parser.h"
#include "changelog.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <sys/stat.h>

#include <microhttpd.h>

int sendTimetable(MHD_Connection* connection)
{
	string html= "<!DOCTYPE html>\n<html><head><title>rmsim</title>\n"
	  "<style>\ntable {\n margin: 0;\n padding: 0;\n border-width: 1px;\n"
	  " border: 1px solid #333;\n border-spacing: 0;\n}\n"
	  "tr {\n background-color: #fff;\n color: #000;\n}\n"
	  "tr:nth-child(odd) {\n background-color: #efd;\n}\n"
	  "th {\n background: #696;\n color: #000;\n margin: 0;\n"
	  " padding: 4px; border: 1px solid #333;\n}\n"
	  "td {\n margin: 0;\n padding: 4px; border: 1px solid #333;\n"
	  " text-align: right;\n}\n"
	  "</style>\n</head>\n<body>\n";
	if (timeTable) {
		string s;
		//timeTable->printTimeSheet2(s);
		timeTable->printTimeSheetHtml(s);
		//html+= "<pre>";
		html+= s;
		//html+= "</pre>";
	} else {
		html+= "No timetable";
	}
	if (interlocking && interlocking->image.size()>0) {
		html+= "<img src=\"/image\">";
	}
	html+= "</body></html>\n";
	MHD_Response* response= MHD_create_response_from_buffer(html.size(),
	  (void*)html.c_str(),MHD_RESPMEM_MUST_COPY);
	if (!response)
		return MHD_NO;
	MHD_add_response_header(response,"Content-Type","text/html");
	int ret= MHD_queue_response(connection,MHD_HTTP_OK,response);
	MHD_destroy_response(response);
	return ret;
}

int sendFile(MHD_Connection* connection, const char* path, const char* mimetype)
{
	int fd= open(path,O_RDONLY);
	if (fd < 0)
		return MHD_NO;
	struct stat statbuf;
	if (fstat(fd,&statbuf) < 0)
		return MHD_NO;
	MHD_Response* response= MHD_create_response_from_fd(statbuf.st_size,fd);
	if (!response)
		return MHD_NO;
	MHD_add_response_header(response,"Content-Type",mimetype);
	int ret= MHD_queue_response(connection,MHD_HTTP_OK,response);
	MHD_destroy_response(response);
	return ret;
}

int handleWebRequest(void* cls, MHD_Connection* connection, const char* url,
  const char* method, const char* version, const char* upload_data,
  size_t* upload_data_size, void** ptr)
{
	if (strcmp(method,"GET") != 0)
		return MHD_NO;
	if (*ptr == NULL) {
		*ptr= (void*)1;
		return MHD_YES;
	}
	if (*upload_data_size != 0)
		return MHD_NO;
	fprintf(stderr,"url %s\n",url);
	if (strcmp(url,"/") == 0)
		return sendTimetable(connection);
	if (strcmp(url,"/image")==0 && interlocking)
		return sendFile(connection,interlocking->image.c_str(),
		  "image/png");
	return MHD_NO;
}
