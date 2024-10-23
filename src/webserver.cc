/*
Copyright © 2021,2024 Doug Jones

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

const char* htmlHead= "<!DOCTYPE html>\n<html><head><title>rmsim</title>\n"
  "<style>\ntable {\n margin: 0;\n padding: 0;\n border-width: 1px;\n"
  " border: 1px solid #333;\n border-spacing: 0;\n}\n"
  "tr {\n background-color: #fff;\n color: #000;\n}\n"
  "tr:nth-child(odd) {\n background-color: #efd;\n}\n"
  "th {\n background: #696;\n color: #000;\n margin: 0;\n"
  " padding: 4px; border: 1px solid #333;\n}\n"
  "td {\n margin: 0;\n padding: 4px; border: 1px solid #333;\n"
  " text-align: right;\n}\n"
  "</style>\n</head>\n<body>\n";

typedef set<string> StrSet;

int sendTimetable(MHD_Connection* connection)
{
	string html= htmlHead;
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

int sendRoutes(MHD_Connection* connection)
{
	string html= htmlHead;
	char* home= getenv("HOME");
	StrSet installs;
	if (home) {
		ulDir* dir= ulOpenDir(home);
		for (ulDirEnt* ent=ulReadDir(dir); ent!=NULL;
		  ent=ulReadDir(dir)) {
			string path= home;
			path+= "/";
			path+= ent->d_name;
			path+= "/ROUTES";
			path= fixFilenameCase(path.c_str());
			if (access(path.c_str(),R_OK))
				continue;
			installs.insert(ent->d_name);
		}
	} else {
		html+= "<p>Cannot find home directory.</p>\n";
	}
	html+= "<p>Select Route:</p><ul>\n";
	for (StrSet::iterator i=installs.begin(); i!=installs.end(); i++) {
		string ipath= home;
		ipath+= "/";
		ipath+= *i;
		ipath+= "/ROUTES";
		ipath= fixFilenameCase(ipath.c_str());
		ulDir* dir= ulOpenDir(ipath.c_str());
		StrSet routes;
		for (ulDirEnt* ent=ulReadDir(dir); ent!=NULL;
		  ent=ulReadDir(dir)) {
			if (ent->d_name[0] == '.')
				continue;
			string path= ipath+"/";
			path+= ent->d_name;
			path+= "/ACTIVITIES";
			path= fixFilenameCase(path.c_str());
			if (access(path.c_str(),R_OK|X_OK))
				continue;
			routes.insert(ent->d_name);
		}
		ulCloseDir(dir);
		for (StrSet::iterator j=routes.begin(); j!=routes.end(); j++) {
			html+= "<li><a href=\"/activities/";
			html+= *i;
			html+= "/ROUTES/";
			html+= *j;
			html+= "\">";
			html+= *i;
			html+= " ";
			html+= *j;
			html+= "</a></li>\n";
		}
	}
	html+= "</ul></body></html>\n";
	MHD_Response* response= MHD_create_response_from_buffer(html.size(),
	  (void*)html.c_str(),MHD_RESPMEM_MUST_COPY);
	if (!response)
		return MHD_NO;
	MHD_add_response_header(response,"Content-Type","text/html");
	int ret= MHD_queue_response(connection,MHD_HTTP_OK,response);
	MHD_destroy_response(response);
	return ret;
}

int sendActivities(MHD_Connection* connection, const char* id)
{
	char* home= getenv("HOME");
	string path= home;
	path+= "/";
	path+= id;
	path+= "/ACTIVITIES";
	path= fixFilenameCase(path.c_str());
	StrSet activities;
	ulDir* dir= ulOpenDir(path.c_str());;
	for (ulDirEnt* ent=ulReadDir(dir); ent!=NULL; ent=ulReadDir(dir)) {
		if (strcasecmp(ent->d_name+strlen(ent->d_name)-4,".act") != 0)
			continue;
		activities.insert(ent->d_name);
	}
	ulCloseDir(dir);
	string html= htmlHead;
	html+= "<p>Select Activity:</p><ul>\n";
	html+= "<li><a href=\"/consists/";
	html+= id;
	html+= "\">Explore</a></li>\n";
	for (StrSet::iterator i=activities.begin(); i!=activities.end(); i++) {
		html+= "<li><a href=\"/briefing/";
		html+= id;
		html+= "/";
		html+= *i;
		html+= "\">";
		html+= *i;
		html+= "</a></li>\n";
	}
	html+= "</ul></body></html>\n";
	MHD_Response* response= MHD_create_response_from_buffer(html.size(),
	  (void*)html.c_str(),MHD_RESPMEM_MUST_COPY);
	if (!response)
		return MHD_NO;
	MHD_add_response_header(response,"Content-Type","text/html");
	int ret= MHD_queue_response(connection,MHD_HTTP_OK,response);
	MHD_destroy_response(response);
	return ret;
}

int sendConsists(MHD_Connection* connection, const char* id)
{
	char* home= getenv("HOME");
	string path= home;
	char* p= strstr((char*)id,"/ROUTES/");
	*p= '\0';
	path+= "/";
	path+= id;
	*p= '/';
	path+= "/TRAINS/CONSISTS";
	path= fixFilenameCase(path.c_str());
	StrSet consists;
	ulDir* dir= ulOpenDir(path.c_str());;
	for (ulDirEnt* ent=ulReadDir(dir); ent!=NULL; ent=ulReadDir(dir)) {
		if (strcasecmp(ent->d_name+strlen(ent->d_name)-4,".con") != 0)
			continue;
		consists.insert(ent->d_name);
	}
	ulCloseDir(dir);
	string html= htmlHead;
	html+= "<p>Select Consist:</p><ul>\n";
	html+= "<li><a href=\"/start/";
	html+= id;
	html+= "/explore\">None</a></li>\n";
	for (StrSet::iterator i=consists.begin(); i!=consists.end(); i++) {
		html+= "<li><a href=\"/start/";
		html+= id;
		html+= "/";
		html+= *i;
		html+= "\">";
		html+= *i;
		html+= "</a></li>\n";
	}
	html+= "</ul></body></html>\n";
	MHD_Response* response= MHD_create_response_from_buffer(html.size(),
	  (void*)html.c_str(),MHD_RESPMEM_MUST_COPY);
	if (!response)
		return MHD_NO;
	MHD_add_response_header(response,"Content-Type","text/html");
	int ret= MHD_queue_response(connection,MHD_HTTP_OK,response);
	MHD_destroy_response(response);
	return ret;
}

string printActHdr(MSTSFileNode* actHdr, const char* field)
{
	string html;
	MSTSFileNode* fnode= actHdr->children->find(field);
	if (fnode) {
		html+= "<p>";
		html+= field;
		html+= ":\n";
		for (MSTSFileNode* line=
		  fnode->getFirstChild();
		  line!=NULL; line=line->next) {
			if (line->value && *(line->value)!="+") {
				html+= "<br>";
				html+= *(line->value);
			}
		}
		html+= "</p>\n";
	}
	return html;
}

int sendActivityDescription(MHD_Connection* connection, const char* id)
{
	string html= htmlHead;
	html+= "<p><a href=\"/start/";
	html+= id;
	html+= "\">Start ";
	html+= id;
	html+= "</a></p>\n";
	char* actid= strrchr((char*)id,'/');
	if (actid) {
		*actid++= '\0';
		char* home= getenv("HOME");
		string path= home;
		char* p= strstr((char*)id,"/ROUTES/");
		*p= '\0';
		path+= "/";
		path+= id;
		path+= "/ROUTES/";
		path+= p+8;;
		path+= "/ACTIVITIES/";
		path+= actid;
		path= fixFilenameCase(path.c_str());
		fprintf(stderr,"act file %s\n",path.c_str());
		MSTSFile file;
		file.readFile(path.c_str());
		MSTSFileNode* act= file.find("Tr_Activity");
		MSTSFileNode* trActHdr=
		  act->children->find("Tr_Activity_Header");
		if (trActHdr != NULL) {
			html+= printActHdr(trActHdr,"Name");
			html+= printActHdr(trActHdr,"Description");
			html+= printActHdr(trActHdr,"Briefing");
		}
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

int sendStartActivity(MHD_Connection* connection, const char* id)
{
	string html= htmlHead;
	html+= "<p>Starting ";
	html+= id;
	html+= "</p>\n";
	char* actid= strrchr((char*)id,'/');
	if (actid) {
		*actid++= '\0';
		char* home= getenv("HOME");
		string path= home;
		char* p= strstr((char*)id,"/ROUTES/");
		*p= '\0';
		path+= "/";
		path+= id;
		fprintf(stderr,"newroute %s %s\n",path.c_str(),p+8);
		mstsRoute= new MSTSRoute(path.c_str(),p+8);
		if (strstr(actid,".con") != 0) {
			mstsRoute->consistName= actid;
			html+= "<p>Position camera near desired track "
			  "and then type \"!start explore\".</p>\n";
		} else if (strncmp(actid,"explore",7) != 0) {
			mstsRoute->activityName= actid;
			path+= "/ROUTES/";
			path+= p+8;;
			path+= "/ACTIVITIES/";
			path+= actid;
			path= fixFilenameCase(path.c_str());
			fprintf(stderr,"act file %s\n",path.c_str());
			MSTSFile file;
			file.readFile(path.c_str());
			MSTSFileNode* act= file.find("Tr_Activity");
			MSTSFileNode* trActHdr=
			  act->children->find("Tr_Activity_Header");
			if (trActHdr != NULL) {
				html+= printActHdr(trActHdr,"Briefing");
			}
		}
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

int send404(MHD_Connection* connection, const char* url)
{
	string html= htmlHead;
	html+= "<p>";
	html+= url;
	html+= " not found</p></body></html>\n";
	MHD_Response* response= MHD_create_response_from_buffer(html.size(),
	  (void*)html.c_str(),MHD_RESPMEM_MUST_COPY);
	if (!response)
		return MHD_NO;
	MHD_add_response_header(response,"Content-Type","text/html");
	int ret= MHD_queue_response(connection,404,response);
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
	if (strcmp(url,"/routes")==0 && mstsRoute==NULL)
		return sendRoutes(connection);
	if (strncmp(url,"/activities/",12)==0 && mstsRoute==NULL)
		return sendActivities(connection,url+12);
	if (strncmp(url,"/consists/",10)==0 && mstsRoute==NULL)
		return sendConsists(connection,url+10);
	if (strncmp(url,"/briefing/",10)==0 && mstsRoute==NULL)
		return sendActivityDescription(connection,url+10);
	if (strncmp(url,"/start/",7)==0 && mstsRoute==NULL)
		return sendStartActivity(connection,url+7);
	return send404(connection,url);
	return MHD_NO;
}
