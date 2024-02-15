import argparse
import xml.etree.ElementTree as ET
import json
import os
import logging

additional_paths_for_all_targets = ["C:/Keil_v5/ARM/ARMCLANG/include",
                                    "C:/Keil_v5/ARM/ARMCC/include"]

class DATA():
    def __init__(self, target_name, include_path, define, outputDir, listingDir, optimLevel):
        self.target_name = target_name
        self.include_path = include_path
        self.define = define
        self.outputDir = outputDir
        self.listingDir = listingDir
        self.optimLevel = optimLevel

def UpdateTasksRelativePaths(relative_path):
    data = ''
    with open('./.vscode/tasks.json') as f:
        data = json.load(f)
        f.close()

    for task in data['tasks']:
        if(len(task['problemMatcher']) == 1):
            continue
        else:
            if(len(task['problemMatcher']['fileLocation']) == 1):
                continue
            else:
                task['problemMatcher']['fileLocation'][1] = "${workspaceFolder}/"+relative_path

    with open('./.vscode/tasks.json', 'w') as f:
        json.dump(data, f, indent = 2)

def UpdateOpenProjectInKeilTask(relative_path):
    data = ''
    with open('./.vscode/tasks.json') as f:
        data = json.load(f)
        f.close()

    for task in data['tasks']:
        if (task['label'] == "Open project in Keil"):
            task['args'][0] = "${workspaceFolder}/"+relative_path

    with open('./.vscode/tasks.json', 'w') as f:
        json.dump(data, f, indent = 2)

def read_keil_project_file(fullFilePath, options, keil_ver):
    if (keil_ver == 4):
        iSkipEverySecondTarget = 0
    tree = ET.parse(fullFilePath)
    root = tree.getroot()
    for target in root.iter('Target'):
        if (keil_ver == 4):
            iSkipEverySecondTarget += 1
            if (iSkipEverySecondTarget % 2 == 0):
                continue
        try:
            target_name = target.find('TargetName').text
        except:
            target_name = None

        try:
            include_path = target.find('TargetOption/TargetArmAds/Cads/VariousControls/IncludePath').text
        except:
            include_path = None

        try:
            define = target.find('TargetOption/TargetArmAds/Cads/VariousControls/Define').text
        except:
            define = None

        try:
            outputDir = target.find('TargetOption/TargetCommonOption/OutputDirectory').text
        except:
            outputDir = None

        try:
            listingDir = target.find('TargetOption/TargetCommonOption/ListingPath').text
        except:
            listingDir = None

        try:
            optimLevel = target.find('TargetOption/TargetArmAds/Cads/Optim').text
        except:
            optimLevel = None

        if(target_name == None):
            target_name = ""

        if(include_path == None):
            include_path = []
        else:
            include_path = include_path.split(';')

        if(define == None):
            define = []
        else:
            define = define.split(',')

        if(outputDir == None):
            outputDir = ""

        if(listingDir == None):
            listingDir = ""

        if(optimLevel == None):
            optimLevel = ""

        options.append(DATA(target_name, include_path, define, outputDir, listingDir, optimLevel))

def ConvertToVsCodeFormat(KeilProjectPath, list):
    for l in list:
        for n, i in enumerate(l.include_path):
            l.include_path[n] = i.replace("\\\\", "/").replace("\\", "/").lstrip()
            if KeilProjectPath is None:
                l.include_path[n] = "${workspaceFolder}/"+l.include_path[n]
            else:
                l.include_path[n] = "${workspaceFolder}/"+KeilProjectPath+"/"+l.include_path[n]
        for a in additional_paths_for_all_targets:
            l.include_path.insert(0, a)
    return list

def KeilDefinesForIntelliSense():
    keil_defines_for_intelli_sense = [
        "__CC_ARM",
        "__arm__",
        "__align(x)=",
        "__ALIGNOF__(x)=",
        "__alignof__(x)=",
        "__asm=",
        "__asm(x)=",
        "__forceinline=",
        "__restrict=",
        "__global_reg(n)=",
        "__inline=",
        "__int64=long long",
        "__INTADDR__(expr)=0",
        "__irq=",
        "__packed=",
        "__pure=",
        "__smc(n)=",
        "__svc(n)=",
        "__svc_indirect(n)=",
        "__svc_indirect_r7(n)=",
        "__value_in_regs=",
        "__weak=",
        "__writeonly=",
        "__declspec(x)=",
        "__attribute__(x)=",
        "__nonnull__(x)=",
        "__register=",
        "__breakpoint(x)=",
        "__cdp(x,y,z)=",
        "__clrex()=",
        "__clz(x)=0U",
        "__current_pc()=0U",
        "__current_sp()=0U",
        "__disable_fiq()=",
        "__disable_irq()=",
        "__dmb(x)=",
        "__dsb(x)=",
        "__enable_fiq()=",
        "__enable_irq()=",
        "__fabs(x)=0.0",
        "__fabsf(x)=0.0f",
        "__force_loads()=",
        "__force_stores()=",
        "__isb(x)=",
        "__ldrex(x)=0U",
        "__ldrexd(x)=0U",
        "__ldrt(x)=0U",
        "__memory_changed()=",
        "__nop()=",
        "__pld(...)=",
        "__pli(...)=",
        "__qadd(x,y)=0",
        "__qdbl(x)=0",
        "__qsub(x,y)=0",
        "__rbit(x)=0U",
        "__rev(x)=0U",
        "__return_address()=0U",
        "__ror(x,y)=0U",
        "__schedule_barrier()=",
        "__semihost(x,y)=0",
        "__sev()=",
        "__sqrt(x)=0.0",
        "__sqrtf(x)=0.0f",
        "__ssat(x,y)=0",
        "__strex(x,y)=0U",
        "__strexd(x,y)=0",
        "__strt(x,y)=",
        "__swp(x,y)=0U",
        "__usat(x,y)=0U",
        "__wfe()=",
        "__wfi()=",
        "__yield()=",
        "__vfp_status(x,y)=0"
    ]

    text = ""

    keil_defines_for_intelli_sense.sort(key=lambda v: v.upper())
    for i, entry in enumerate(keil_defines_for_intelli_sense):
        text += f"        \"{entry}\""
        if (i == len(keil_defines_for_intelli_sense) - 1):
            text += "\n"
        else:
            text += ",\n"
    
    return text

def WriteJsonFile(list, path):
    text = "{\n  \"configurations\": [\n"
    for list_cnt, list_ele in enumerate(list):
        text += "    {\n"
        text += "      \"name\": \""+list_ele.target_name+"\",\n"
        text += "      \"includePath\": [\n"
        for cnt, ele in enumerate(list_ele.include_path):
            text += "        \""+ele+"\""
            if(cnt == len(list_ele.include_path)-1):
                text += "\n"
            else:
                text += ",\n"
        text += "      ],\n"
        text += "        \"defines\": [\n"
        for cnt, ele in enumerate(list_ele.define):
            text += "        \""+ele.replace(" ", "")+"\""
            if(cnt == len(list_ele.define)-1):
                text += ",\n"
                text += KeilDefinesForIntelliSense()
            else:
                text += ",\n"
        text += "      ]\n"
        if(list_cnt == len(list)-1):
            text += "    }\n"
        else:
            text += "    },\n"
    text += "  ],\n  \"version\": 4\n}"

    json_file = open(path, "w")
    json_file.write(text)
    json_file.close()

def WriteParseKeilProjectBatchFile(KeilProjectPath, KeilProjectFilename):
    text = "@echo off\n"
    text += "\n"
    text += "call Scripts\_errorlevel_handler.bat"
    text += "\n"
    if KeilProjectPath is None:
        text += f'SET KEIL_PROJECT_PATH={KeilProjectFilename}\n'
    else:
        text += f'SET KEIL_PROJECT_PATH={KeilProjectPath}\\{KeilProjectFilename}\n'
    text += f'python ./Scripts/_parse_Keil_project.py "%KEIL_PROJECT_PATH%" .vscode/c_cpp_properties.json ./Scripts/_set_Keil_project_paths.bat\n'
    text += f'call Scripts\\_set_Keil_project_paths.bat\n'
    text += f'%ifErr% echo( &(%errExit%)\n'

    batch_file = open("./Scripts/_parse_Keil_project.bat", "w")
    batch_file.write(text)
    batch_file.close()

def WriteKeilProjectPathBatchFile(KeilProjectPath, KeilProjectFilename, bat_file_path, list):
    text = "@echo off\n"
    text += "\n"

    if KeilProjectPath is None:
        text += "SET PROJECT_FOLDER=.\n"
    else:
        KeilProjectPath = KeilProjectPath.replace("/", "\\")
        text += "SET PROJECT_FOLDER="+KeilProjectPath+"\n"
    text += "SET KEIL_PROJECT_NAME_UVPROJX="+KeilProjectFilename+"\n"
    text += "\n"

    text += "SET PRODUCTION_FOLDER=Production\n"
    text += "SET TMP_FOLDER=ci_tmp\n"
    text += "SET BUILD_LOG=build.log\n"
    text += "\n"

    index = 0
    for l in list:
        text += "IF \"%PROJECT_TARGET%\"==\""+l.target_name+"\" (\n"
        text += "\tSET KEIL_PROJECT_OUTPUT_DIR="+l.outputDir+"\n"
        text += "\tSET KEIL_PROJECT_LISTING_DIR="+l.listingDir+"\n"
        text += "\tSET KEIL_PROJECT_OPTIM_LEVEL="+l.optimLevel+"\n"
        text += ")\n"
        text += "SET PROJECT_TARGET_LIST["+str(index)+"]="+str(l.target_name)+"\n"
        text += "SET KEIL_PROJECT_OUTPUT_DIR_LIST["+str(index)+"]="+str(l.outputDir)+"\n"
        text += "SET KEIL_PROJECT_LISTING_DIR_LIST["+str(index)+"]="+str(l.listingDir)+"\n"
        text += "SET KEIL_PROJECT_OPTIM_LEVEL_LIST["+str(index)+"]="+str(l.optimLevel)+"\n"
        index += 1
        text += "\n"

    text += "SET PROJECT_TARGET_LIST_LENGTH="+str(index - 1)+"\n"

    batch_file = open(bat_file_path, "w")
    batch_file.write(text)
    batch_file.close()

def PrintData(list):
    for l in list:
        print("Target name:", l.target_name)
        print("  Include_path:")
        for e in l.include_path:
            print("    ", e)
        print("  Define:")
        for e in l.define:
            print("    ", e)

if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('keil_project_file_path')
    PARSER.add_argument('json_file_path')
    PARSER.add_argument('bat_file_path')
    APP_ARGS = PARSER.parse_args()
    APP_ARGS.keil_project_file_path = APP_ARGS.keil_project_file_path.replace("\\", "/")

    path_Filename = APP_ARGS.keil_project_file_path.rsplit("/", 1)
    if len(path_Filename) > 1:
        keil_project_path = path_Filename[0]
        keil_project_filename = path_Filename[1]
    else:
        keil_project_path = None
        keil_project_filename = path_Filename[0]

    keil_project_options = []

    logging.info(f'Parsing "{APP_ARGS.keil_project_file_path}"')
    if APP_ARGS.keil_project_file_path.endswith('.uvproj'):
        keil_ver = 4
        read_keil_project_file(APP_ARGS.keil_project_file_path, keil_project_options, keil_ver)
    elif APP_ARGS.keil_project_file_path.endswith('.uvprojx'):
        keil_ver = 5
        read_keil_project_file(APP_ARGS.keil_project_file_path, keil_project_options, keil_ver)
    else:
        logging.error(f"\"{APP_ARGS.keil_project_file_path}\" is not a Keil's project file")
        exit(1)

    keil_project_options = ConvertToVsCodeFormat(keil_project_path, keil_project_options)
    WriteKeilProjectPathBatchFile(keil_project_path, keil_project_filename, APP_ARGS.bat_file_path, keil_project_options)
    WriteParseKeilProjectBatchFile(keil_project_path, keil_project_filename)
    WriteJsonFile(keil_project_options, APP_ARGS.json_file_path)
    UpdateTasksRelativePaths(os.path.dirname(APP_ARGS.keil_project_file_path))
    UpdateOpenProjectInKeilTask(APP_ARGS.keil_project_file_path)
    logging.info("Done")
