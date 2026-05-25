import json
import pandas as pd
from evaluateUtils.config import KB_DIR
import os


def load_json(json_file: str) -> dict:
    with open(json_file, 'r', encoding='utf-8') as file:
        file = json.load(file)
    return file


def sort_single_dimension_tag_code(tag_code_str: str) -> str:
    """
    Sort the tag code in a single dimension.
    """
    tag_code_list = tag_code_str.split(';')
    tag_code_list.sort()
    return ';'.join(tag_code_list)


def remove_lst_duplicates_with_order(lst: list) -> list:
    """
    Remove duplicate elements from list while preserving their order.
    """
    seen = set()
    seen_add = seen.add
    return [x for x in lst if not (x in seen or seen_add(x))]


def is_lefthand_scn(scn_name: str) -> bool:
    """
    Judge whether the given scenario is a lefthand-rule scenario.
    """
    # Method 1: record the lefthand scenario name in a csv file
    lefthand_path = os.path.join(KB_DIR, 'lefthand_flag.csv')
    lefthand_df = pd.read_csv(lefthand_path)
    return scn_name in lefthand_df['scenario'].values



