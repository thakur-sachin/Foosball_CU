#pragma once
#include "pugixml.hpp"
#include "dictionary.h"

class UserDefaults
{

public:

	typedef enum _xmlParseErrors {
		XML_ERR_NONE = 0,
		XML_ERR_FILE_OPEN,
		XML_ERR_DICT_CREATE,
		XML_ERR_COMMON,
		XML_ERR_CORE,
		XML_ERR_PWR_CFG,
		XML_ERR_MODEL_CFG,
		XML_ERR_ENH_OPT_CFG,
		XML_ERR_ADV_OPT_CFG
	} xmlParseError;

	typedef enum _ParamLoadTypes {
		PARAM_LOAD_COMMON,
		PARAM_LOAD_TUNE,
		PARAM_LOAD_CONFIG
	} ParamLoadType;

private:
	const int MODEL_LOC = 4;

    //ML-SCHP-310
	const int ML_MODEL_LOC = 3;
	const int ML_CORE_LOC = 8;
    const int ML_END_LOC = 10;

	//CPM-SCHP-3411P-ELSA
	const int CPM_CORE_LOC = 9;
	const int CPM_PWR_LOC = 12;
	const int CPM_WIND_LOC = 13;
	const int CPM_ENH_LOC = 15;
	const int CPM_ADV_LOC = 18;

	//CPM-SCHP-N0561P-ELSA
	const int ACG_CORE_LOC = 9;
	const int ACG_PWR_LOC = 13;
	const int ACG_WIND_LOC = 14;
	const int ACG_ENH_LOC = 16;
	const int ACG_ADV_LOC = 19;


	pugi::xml_document user_settings_document;
	pugi::xml_parse_result result;
	pugi::xml_node tuneDB;
	pugi::xml_node keys;
	pugi::xml_node commonSettings;
	pugi::xml_node model;
	pugi::xml_node pwrSect;
	pugi::xml_node cfg;
	
	bool loaded;
	xmlParseError lastErr;

	pugi::xml_attribute attributes;

	void LoadAttributesFromElement(dictionary * d, pugi::xml_node sect);
	void LoadElementAndChildHwSect(dictionary * d, pugi::xml_node sect, int hwCode);

	int NullPowerCode(char code[]);


public:

	void load(char* filename);
	void unload();

	dictionary * LookupCommonSettings(char key[], int hwCode);
	dictionary * LookupTuningSettings(char pn[], int hwCode, ParamLoadType loadType);
	
	xmlParseError GetLastError();

	bool Loaded();

	// Create an unloaded instance
	UserDefaults();
	// Cleanup
	~UserDefaults();
};