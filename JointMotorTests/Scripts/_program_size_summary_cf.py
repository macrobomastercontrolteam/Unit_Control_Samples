import argparse
import re
import os
import math
from terminaltables import AsciiTable

def parse_log_cf(log_path):
    bl_size = 0
    app_size = 0
    used_memory_size = 0
    max_size = 0
    total_memory_size = 0

    filename =  log_path
    ifile = open(filename, 'r')
    lines = ifile.readlines()
    ifile.close()

    filename_cf = os.path.splitext(filename)[0] + f'_cf.log'
    ofile = open(filename_cf, 'w')
    table_data = [
        ['Target', 'Build time', 'BL Size [B]', 'APP Size [B]', 'BL+APP Size [B]', 'Total FLASH Size [B]', 'Memory used [%]', 'Memory left [B]']
    ]
    table_data_entry = []
    for line in lines:
        target_name_search = re.search("^Build summary for (.+?) \(.+\) $", line)
        build_time_search = re.search("^Build summary for .+? \((.+)\) $", line)
        app_size_search = re.search("^.*\.map:.*\)\s*(\d*) \(.*$", line)
        bl_max_size_search = re.search("Load Region LR_IROM1.*Base: 0x.{4}(\S+),.*Max: (0x\S+),", line)
        if target_name_search :
            table_data_entry.append(f'{target_name_search.group(1)}');
        if build_time_search :
            table_data_entry.append(f'{build_time_search.group(1)}');
        if app_size_search:
            app_size = int(app_size_search.group(1))
        if bl_max_size_search:
            bl_size = int(f'0x0000{bl_max_size_search.group(1)}',16)
            max_size = int(bl_max_size_search.group(2),16)
            total_memory_size = 2**math.ceil(math.log2(max_size + bl_size - 1))
            used_memory_size = bl_size + app_size
            table_data_entry.append(f'{bl_size}');
            table_data_entry.append(f'{app_size}');
            table_data_entry.append(f'{used_memory_size}');
            table_data_entry.append(f'{total_memory_size}');

            memory_used_percent = used_memory_size/total_memory_size * 100
            table_data_entry.append(f'{memory_used_percent:.2f}');

            table_data_entry.append(f'{total_memory_size - used_memory_size}');
            table_data.append(table_data_entry)
            table_data_entry = []
    table = AsciiTable(table_data)
    for i in range(2, len(table.column_widths)):
        table.justify_columns[i] = 'right'
    ofile.write(table.table)
    ofile.close()
  
if __name__ == "__main__":
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('log_path')
    APP_ARGS = PARSER.parse_args()
    APP_ARGS.log_path = APP_ARGS.log_path.replace("\\", "/")

    parse_log_cf(APP_ARGS.log_path)

    