from typing import List, Tuple, Optional
import pandas as pd
import os
from evaluateUtils.config import ROAD_TAG_CODE_COL, INFR_TAG_CODE_COL, MAN_TAG_CODE_COL, ENV_TAG_CODE_COL, KB_DIR, SCENARIO_ID_COL, SEGMENT_ID_COL, ROAD_TAG_COL, INFR_TAG_COL, MAN_TAG_COL, ENV_TAG_COL
from evaluateUtils.TrafficRule_compliance.file_utils import sort_single_dimension_tag_code
from evaluateUtils.TrafficRule_compliance.TagTree import TagTree

def load_database(anchor: str, filename_suffix: str = '.csv') -> Tuple[pd.DataFrame, dict]:
    """
    Load the traffic rule knowledgebase from the file.
    """
    filename = anchor + filename_suffix
    file_path = os.path.join(KB_DIR, filename)
    if not os.path.exists(file_path):
        raise FileNotFoundError(f'The input file {filename} does not exist.')
    df = pd.read_csv(file_path)
    # fill the nan with '/root'
    df.fillna('', inplace=True)
    # check whether the tagcode in the database is correct
    tree = TagTree()
    for i in range(df.shape[0]):
        tmp = [tree.find_tag_by_code(code) for code in df.loc[i, ROAD_TAG_CODE_COL].split(';')]
        tmp = [tree.find_tag_by_code(code) for code in df.loc[i, INFR_TAG_CODE_COL].split(';')]
        tmp = [tree.find_tag_by_code(code) for code in df.loc[i, MAN_TAG_CODE_COL].split(';')]
        tmp = [tree.find_tag_by_code(code) for code in df.loc[i, ENV_TAG_CODE_COL].split(';')]
    del tree
    return df


def load_multi_database(db_list: List[str], filename_suffix: Optional[List[str]] = None) -> Tuple[pd.DataFrame, dict]:
    """
    Given a list of database names, load the traffic rule databases from the files.
    """
    if filename_suffix is None:
        filename_suffix = ['.csv'] * len(db_list)
    db = pd.DataFrame()
    for i in range(len(db_list)):
        df = load_database(anchor=db_list[i], filename_suffix=filename_suffix[i])
        db = pd.concat([db, df], ignore_index=True)
    del df
    tag2rule_dict = {}
    for i in range(db.shape[0]):
        # If more than one tag code exists in the same dimension, they should be sorted.
        all_tag = ';'.join([
            sort_single_dimension_tag_code(str(db.loc[i, ROAD_TAG_CODE_COL])),
            sort_single_dimension_tag_code(str(db.loc[i, INFR_TAG_CODE_COL])),
            sort_single_dimension_tag_code(str(db.loc[i, MAN_TAG_CODE_COL])),
            sort_single_dimension_tag_code(str(db.loc[i, ENV_TAG_CODE_COL]))
        ])
        if all_tag not in tag2rule_dict.keys():
            tag2rule_dict[all_tag] = []
        tag2rule_dict[all_tag].append(i)

    return db, tag2rule_dict


def load_tag_names(scn_tag: pd.DataFrame) -> pd.DataFrame:
    """
    Load the scenario tags' names.
    """
    # wth:加载文件取消
    # file_path = os.path.join(CACHE_DIR, input_filename)
    # if file_path.endswith('.xlsx'):
    #     scn_tag = pd.read_excel(file_path)
    # elif file_path.endswith('.csv'):
    #     scn_tag = pd.read_csv(file_path)
    # else:
    #     raise ValueError("The input file should be either an excel file or a csv file")
    if SCENARIO_ID_COL not in scn_tag.columns:
        raise ValueError("The input file should contain the ID of the given scenario")
    if SEGMENT_ID_COL not in scn_tag.columns:
        raise ValueError("The input file is supposed to be a segmented scenario, containing the segment ID")
    scn_tag[SCENARIO_ID_COL] = scn_tag[SCENARIO_ID_COL].astype(str)
    scn_tag[SEGMENT_ID_COL] = scn_tag[SEGMENT_ID_COL].astype(str)
    if not set([ROAD_TAG_COL, INFR_TAG_COL, MAN_TAG_COL, ENV_TAG_COL]).issubset(set(list(scn_tag.columns))):
        raise ValueError("The input file should contain the four tag name columns")
    # transfom tag name columns to string, and then fill the nan with ''
    scn_tag[ROAD_TAG_COL] = scn_tag[ROAD_TAG_COL].fillna('')
    scn_tag[INFR_TAG_COL] = scn_tag[INFR_TAG_COL].fillna('')
    scn_tag[MAN_TAG_COL] = scn_tag[MAN_TAG_COL].fillna('')
    scn_tag[ENV_TAG_COL] = scn_tag[ENV_TAG_COL].fillna('')

    # only return the columns that are needed
    return scn_tag[[SCENARIO_ID_COL, SEGMENT_ID_COL, ROAD_TAG_COL, INFR_TAG_COL, MAN_TAG_COL, ENV_TAG_COL]]


def tag2tagcode(tag_names: pd.DataFrame) -> pd.DataFrame:
    """
    transform the tag to the tag-code by finding the tag name in the tag-tree.
    """
    tree = TagTree()

    def t2tc(t: str, tree: TagTree) -> str:
        if not t:
            return '/root'
        t_list = [item.strip() for item in t.split(';') if item.strip()]
        tagcode_list = []
        for target_name in t_list:
            nodes = tree.find_tag(target_name=target_name)
            if len(nodes) == 1:
                tagcode_list.append(nodes[0].code)
            elif len(nodes) > 1:
                print(f'Error in tag tree. {len(nodes)} node were found when given {target_name}.')
            else:
                # print('tag name list:',t_list)
                print('Error tag name:', target_name)
        return ';'.join(tagcode_list)

    for i in range(tag_names.shape[0]):
        road_t = tag_names.loc[i, ROAD_TAG_COL]
        infr_t = tag_names.loc[i, INFR_TAG_COL]
        man_t = tag_names.loc[i, MAN_TAG_COL]
        env_t = tag_names.loc[i, ENV_TAG_COL]

        tag_names.loc[i, ROAD_TAG_CODE_COL] = t2tc(road_t, tree)
        tag_names.loc[i, INFR_TAG_CODE_COL] = t2tc(infr_t, tree)
        tag_names.loc[i, MAN_TAG_CODE_COL] = t2tc(man_t, tree)
        tag_names.loc[i, ENV_TAG_CODE_COL] = t2tc(env_t, tree)
    del tree
    return tag_names


def load_scenario_elements(scn_tag: pd.DataFrame, traffic_rule_query_type: str = 'tagcode') -> pd.DataFrame:
    """
    Load the scenario elements.
    First, the tag-names are loaded.
    After that, the tag names will be transformed to tag-code to support the following retrieval.
    When using "tagcode" as the query type, the tag names will be transformed to tag-code.
    When other query types are given, an error will be raised.
    """
    tag_names = load_tag_names(scn_tag)
    if traffic_rule_query_type == 'tagcode':
        scn_elm = tag2tagcode(tag_names=tag_names)
    else:
        raise ValueError("The traffic_rule_query_type must be 'tagcode'")
    return scn_elm


def filter_scenario_tags(scn_tag: pd.DataFrame, tag2rule_dict: dict) -> pd.DataFrame:
    """
    In order to accelerate the retrieval process, useless (specifically, not in the database) tag codes will be filtered.
    """
    def filter_single_tag(raw_tag: str, all_tags: List[str]) -> str:
        raw_tag_list = [item.strip() for item in raw_tag.split(';') if item.strip()]
        filtered_tag_list = [item for item in raw_tag_list if item in all_tags]
        # if len(filtered_tag_list)<len(raw_tag_list):
        # print('The following tag code is not in the database:',set(raw_tag_list)-set(filtered_tag_list))
        return ';'.join(filtered_tag_list) if filtered_tag_list else '/root'

    all_tags_str = list(tag2rule_dict.keys())
    all_tags = []
    for tag_str in all_tags_str:
        all_tags.extend(tag_str.split(';'))
    all_tags = list(set(all_tags))

    for i in range(scn_tag.shape[0]):
        scn_tag.loc[i, ROAD_TAG_CODE_COL] = filter_single_tag(
            raw_tag=scn_tag.loc[i, ROAD_TAG_CODE_COL],
            all_tags=all_tags,
        )
        scn_tag.loc[i, INFR_TAG_CODE_COL] = filter_single_tag(
            raw_tag=scn_tag.loc[i, INFR_TAG_CODE_COL],
            all_tags=all_tags,
        )
        scn_tag.loc[i, MAN_TAG_CODE_COL] = filter_single_tag(
            raw_tag=scn_tag.loc[i, MAN_TAG_CODE_COL],
            all_tags=all_tags,
        )
        scn_tag.loc[i, ENV_TAG_CODE_COL] = filter_single_tag(
            raw_tag=scn_tag.loc[i, ENV_TAG_CODE_COL],
            all_tags=all_tags,
        )
    return scn_tag

