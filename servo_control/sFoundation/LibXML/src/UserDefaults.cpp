#include "pugixml.hpp"
#include "UserDefaults.hpp"

UserDefaults::UserDefaults() {
	loaded = false;
}

// Return all memory
UserDefaults::~UserDefaults() {
	unload();
#if (defined(_WIN32) || defined(_WIN64))
	_RPT0(_CRT_WARN, "UserDefaults deleted\n");
#endif
}

void UserDefaults::load(char* filename) {
	result = user_settings_document.load_file(filename);
#if (defined(_WIN32) || defined(_WIN64))
	_RPT1(_CRT_WARN, "Load user XML file result: %s\n", result.description());
#endif
	if (result.status) {
		lastErr = XML_ERR_FILE_OPEN;
	} else {
		// only do this if the file is open successfully
		tuneDB = user_settings_document.child("TuningDB");
		keys = tuneDB.child("Keys");
		//for (pugi::xml_node key = keys.child("Key"); key; key = key.next_sibling("Key"))
		//{
		//	_RPT1(_CRT_WARN, "Iterator Keys: %s\n", key.attribute("Motor").value());
		//}
		loaded = true;
	}
}

void UserDefaults::unload() {
	loaded = false;
	// free the XML file by loading an invalid one
	user_settings_document.load_file("");
}

dictionary * UserDefaults::LookupCommonSettings(char key[], int hwCode) {
	dictionary * dict;

	lastErr = XML_ERR_NONE;

	if (!loaded) {
		lastErr = XML_ERR_FILE_OPEN;
		return NULL;
	}

	dict = dictionary_new(0);
	if (!dict) {
		lastErr = XML_ERR_DICT_CREATE;
	}

	commonSettings = keys.find_child_by_attribute("Motor", key);
	model = commonSettings.child("Model");
	if (!model) {
		lastErr = XML_ERR_COMMON;
	}

	LoadElementAndChildHwSect(dict, model, hwCode);

	return dict;
}

dictionary *  UserDefaults::LookupTuningSettings(char pn[], int hwCode, ParamLoadType loadType) {
	pugi::xml_node modelSettings;
	pugi::xml_node hwSettings;
	char coreStr[10], modelStr[10], *cfgSect;
	char pwrStr[10], enhOptStr[10], advOptStr[10], scStr[4];
	
	bool isSC = false;

	bool isEtherPath = false;

	int pwrCode;

	lastErr = XML_ERR_NONE;

	if (loadType == PARAM_LOAD_TUNE) {
		cfgSect = "CfgTune";
	}
	else {
		cfgSect = "Cfg";
	}

	// get model settings and resolve the SC option
    if (hwCode == 2) {
        snprintf(modelStr, sizeof(modelStr), "%.4s", &pn[ML_MODEL_LOC]);
    }
    else {
        snprintf(modelStr, sizeof(modelStr), "%.4s", &pn[MODEL_LOC]);
    }
	snprintf(scStr, sizeof(scStr), "%.2s", modelStr);
	isSC = !strcmp(scStr, "SC");
	isEtherPath = !strcmp(scStr, "EC");

	// build up the XML core lookup string from the part number
    switch (hwCode) {
        case 0: // ClearPath
		case 3: // EtherPath
            // 23xx/34xx core identifier - 341XS
            snprintf(coreStr, sizeof(coreStr), "%.3sX%.1s", &pn[CPM_CORE_LOC], &pn[CPM_WIND_LOC]);
            snprintf(enhOptStr, sizeof(enhOptStr), "CPOpt_%.2s%.1s", &pn[MODEL_LOC], &pn[CPM_ENH_LOC]);

            if (isSC || isEtherPath)
                snprintf(advOptStr, sizeof(advOptStr), "SCOpt_%.1s", &pn[CPM_ADV_LOC]);

            // Grab the power indicator
            if (strcmp(modelStr, "NULL") == 0) {
                char tempStr[4];
                snprintf(tempStr, sizeof(tempStr), "%.3s", &pn[CPM_CORE_LOC]);
                pwrCode = NullPowerCode(tempStr);
            }
            else {
                pwrCode = atoi(&pn[CPM_PWR_LOC]);
            }
            break;
        case 1: // ACGreen
            // ACG core identifier - N056P
            if (pn[ACG_WIND_LOC] == 'P' || pn[ACG_WIND_LOC] == 'A')
                // "P" and "A" windings are interchangeable; use the "P" version for the lookup
                snprintf(coreStr, sizeof(coreStr), "%.4sP", &pn[ACG_CORE_LOC]);
            else
                snprintf(coreStr, sizeof(coreStr), "%.4s%.1s", &pn[ACG_CORE_LOC], &pn[ACG_WIND_LOC]);

            snprintf(enhOptStr, sizeof(enhOptStr), "CPOpt_%.2s%.1s", &pn[MODEL_LOC], &pn[ACG_ENH_LOC]);

            if (isSC)
                snprintf(advOptStr, sizeof(advOptStr), "SCOpt_%.1s", &pn[ACG_ADV_LOC]);

            // Grab the power indicator
            pwrCode = atoi(&pn[ACG_PWR_LOC]);
            break;
        case 2: // MicroLoop
            // MicroLoop core has no "power rating" or winding specification, we only
            // need the core #
            snprintf(coreStr, sizeof(coreStr), "%.3s", &pn[ML_CORE_LOC]);
            // MicroLoop is Enhanced by default, setup the enhOptStr
            snprintf(enhOptStr, sizeof(enhOptStr), "CPOpt_SCE");
            // MicroLoop is Advanced by default, setup the advOptStr
            snprintf(advOptStr, sizeof(advOptStr), "SCOpt_A");
            pwrCode = 0;
            break;
    }

    // build the power section key
    if (strstr(modelStr, "SK") != NULL) {
        snprintf(pwrStr, sizeof(pwrStr), "PWR%d_SK", pwrCode);
    }
    else {
        snprintf(pwrStr, sizeof(pwrStr), "PWR%d", pwrCode);
    }

	dictionary * dict;
	// create the dictionary to hold data from all relevant configuration settings
	dict = dictionary_new(0);
	if (!dict) {
		lastErr = XML_ERR_DICT_CREATE;
	}

	if (!loaded) {
		lastErr = XML_ERR_FILE_OPEN;
		return NULL;
	}

	// get a link to the core settings (341XS/N056P...)
	commonSettings = keys.find_child_by_attribute("Motor", coreStr);
	
	// get a link to the current power level setting within the core section
	pwrSect = commonSettings.child(pwrStr);
	if(!pwrSect) {
		lastErr = XML_ERR_PWR_CFG;
	}

	// we are interested in the configuration attributes 
	cfg = pwrSect.child(cfgSect);
	
	if (!pwrSect && !cfg) {
		lastErr = XML_ERR_PWR_CFG;
	}
	LoadElementAndChildHwSect(dict, cfg, hwCode);

	// load the model specific configuration settings (SCHP/MCPV...)
	modelSettings = keys.find_child_by_attribute("Motor", modelStr);
	// load config section
	cfg = modelSettings.child(cfgSect);
	if (!modelSettings && !cfg) {
		lastErr = XML_ERR_MODEL_CFG;
	}
	LoadElementAndChildHwSect(dict, cfg, hwCode);
	
    // load the E vs R configuration settings
    modelSettings = keys.find_child_by_attribute("Motor", enhOptStr);
    // load config section
    cfg = modelSettings.child(cfgSect);
    if (!modelSettings && !cfg) {
        lastErr = XML_ERR_ENH_OPT_CFG;
    }
    LoadElementAndChildHwSect(dict, cfg, hwCode);
    
    if (isSC || isEtherPath) {
        modelSettings = keys.find_child_by_attribute("Motor", advOptStr);

        cfg = modelSettings.child(cfgSect);
        if (!modelSettings && !cfg) {
            lastErr = XML_ERR_ADV_OPT_CFG;
        }
        LoadElementAndChildHwSect(dict, cfg, hwCode);
    }

	return dict;
}

void UserDefaults::LoadAttributesFromElement(dictionary * d, pugi::xml_node sect) {
	int  errs = 0;
	for (pugi::xml_attribute_iterator ait = sect.attributes_begin(); ait != sect.attributes_end(); ++ait)
	{
		errs = dictionary_set(d, ait->name(), ait->value());
#if (defined(_WIN32) || defined(_WIN64))
		// read out item just saved
		_RPT2(_CRT_WARN, "Key and value in dictionary: %s = %s\n", ait->name(), dictionary_get(d, ait->name(), 0));
		_RPT2(_CRT_WARN, "attributes: %s = %s\n", ait->name(), ait->value());
#endif
	}

}

void  UserDefaults::LoadElementAndChildHwSect(dictionary * d, pugi::xml_node sect, int hwCode) {
	char hwStr[8];

	// load the section attributes
	LoadAttributesFromElement(d, sect);

	// build up the hardware settings key
	snprintf(hwStr, sizeof(hwStr), "CPHW%d", hwCode);

	// load the HW specific attributes
	LoadAttributesFromElement(d, sect.child(hwStr));
}

UserDefaults::xmlParseError UserDefaults::GetLastError() {
	return lastErr;
}

bool UserDefaults::Loaded() {
	return loaded;
}

int UserDefaults::NullPowerCode(char code[]) {
	// Return the appropriate power code for a null motor of a specific size
	if (strcmp(code, "343") == 0) {
		return 2;
	}
	else {
		return 1;
	}
}
