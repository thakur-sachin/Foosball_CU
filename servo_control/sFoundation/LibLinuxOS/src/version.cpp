/*
 * version.cpp
 *
 *  Created on: Jun 23, 2016
 *      Author: dave
 *
 *  Extract version information about a file. This is mostly a Windows thing for now.
 */


#include "version.h"
#include <libgen.h>
#include <unistd.h>
#include <string.h>

CVersionInfo::CVersionInfo(void)
{
	m_bValidPath = false;
	m_bVersionInfoExists = false;
	m_nLanguage = 0;
	m_nLangCount = 0;
	char fPath[300];
	fPath[0]=0;
	int nRead = readlink("/proc/self/exe", fPath, sizeof(fPath)-1);
	if (nRead != -1) {
		fPath[nRead]=0;
		m_pszFilePath = fPath;
	}
}


CVersionInfo::~CVersionInfo(void)
{

}


bool CVersionInfo::SetFilePath(string szPath)
{
	return false;
}


bool CVersionInfo::SetLanguage(int nLanguage)
{
	return false;
}


int CVersionInfo::GetLanguageCount(void)
{
	return 0;
}


string CVersionInfo::GetComments(void)
{
	return "N/A";
}


string CVersionInfo::GetCompanyName(void)
{
	return "N/A";
}


string CVersionInfo::GetFileDescription(void)
{
	return "N/A";
}


string CVersionInfo::GetFileVersion(void)
{
	return "N/A";
}


string CVersionInfo::GetInternalName(void)
{
	return "N/A";
}


string CVersionInfo::GetLegalCopyright(void)
{
	return "N/A";
}


string CVersionInfo::GetLegalTrademarks(void)
{
	return "N/A";
}


string CVersionInfo::GetOriginalFilename(void)
{
	return "N/A";
}


string CVersionInfo::GetProductName(void)
{
	return "N/A";
}


string CVersionInfo::GetProductVersion(void)
{
	return "N/A";
}


string CVersionInfo::GetPrivateBuild(void)
{
	return "N/A";
}
string CVersionInfo::GetSpecialBuild(void)
{
	return "N/A";
}

string CVersionInfo::GetFileDir()
{
	char theFilePath[300];
	strncpy(theFilePath, m_pszFilePath.c_str(), sizeof(theFilePath)-1);
	theFilePath[299] = 0;
	return dirname(theFilePath);
}

