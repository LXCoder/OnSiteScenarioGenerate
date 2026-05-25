import os
from typing import List, Tuple
import itertools
from evaluateUtils.config import CONTENT_COL, ARTICLE_COL, SOURCE_COL
import pandas as pd

class MetaTrafficRule:
    """
    Represents a meta traffic rule.

    Attributes:
        content (str): The content of the traffic rule.
        source (str): The source of the traffic rule.
        article (str): The article related to the traffic rule.

    Methods:
        embed_content(semb_model): Embeds the content of the traffic rule using the specified semantic model.
        embed_ConcatTags(semb_model): Embeds the concatenated tags of the traffic rule using the specified semantic model.
        embed_EachTag(semb_model): Embeds each individual tag of the traffic rule using the specified semantic model.
    """

    def __init__(self, content, source, article):
        self.content = str(content)
        self.source = str(source)
        self.article = str(article)

    def embed_content(self, semb_model):
        return semb_model.encode(self.content, normalize_embeddings=True)

    def show(self):
        print('source:', self.source)
        print('article:', self.article)
        print('content:', self.content)



def get_all_directories(code: str) -> list:
    """
    Get all directories corresponding to the specified tag code.
    Args:
        code (str): The tag code string, which should start with '/root'.
    Returns:
        list: A list containing all directories, from the root directory to the directory where the specified tag code is located, in path order.
    Raises:
        ValueError: If the input tag code does not start with '/root', an exception is raised.
    """
    if code == '':
        return []
    if not code.startswith('/root'):
        raise ValueError(f'Input tag code:{code}\nIllegal tag code! A correct tag code should be start with "/root"')
    directories = []
    while code != '/':
        directories.append(code)
        code = os.path.dirname(code)
    directories.reverse()  # 反转列表顺序
    return directories


def is_substring(small, large):
    return small in large


def get_combinations_without_substrings(elements: List[str]) -> List[str]:
    """
    Get all combinations of given elements without any substring relationships within the combinations.

    Args:
        elements (List[str]): A list of elements (strings) to generate combinations from.

    Returns:
        List[str]: A list of all valid combinations, where each combination is represented as a string(split by ';').
        No string in a combination is a substring of any other string in the same combination.
    """
    # elements.remove('/root')
    valid_combinations = []
    for r in range(2, len(elements) + 1):
        for combo in itertools.combinations(elements, r):
            if not any(is_substring(combo[i], combo[j]) or is_substring(combo[j], combo[i]) \
                       for i in range(len(combo)) for j in range(i + 1, len(combo))):
                # when combined tags are employed, make sure the order of the tags is consistent. 
                # Specifically, make sure 'A;B', instead of 'B;A', is the only format when A and B are combined.
                # sort the combo
                c = list(combo)
                c.sort()
                valid_combinations.append(c)
    valid_combinations = [';'.join(item) if not isinstance(item, str) else item for item in valid_combinations]
    return valid_combinations


def get_single_dimension_query_list(original_tag: List[str]) -> list:
    """
    Generate a single-dimension query list from the given original tag list.
    Args:
        original_tag (list): The original list of tags, containing multiple string-type tags.
    Returns:
        list: The processed single-dimension query list, containing multiple string-type tags.
    Raises:
        ValueError: Raised if the input original tag list is empty.
    """
    tag_lst = []
    # step 1: get hierarchical tags
    for tag in original_tag:
        tag_lst.extend(get_all_directories(tag))
    # drop duplicates to accelerate step 2
    tag_lst = list(set(tag_lst))

    # step 2: get combined tags which are sibling nodes
    combine_tag_lst = []
    if len(original_tag) > 1:
        combine_tag_lst = get_combinations_without_substrings(tag_lst)

    # step 3: add two types of tags
    tag_lst.extend(combine_tag_lst)
    # drop duplicates
    tag_lst = list(set(tag_lst))
    if len(tag_lst) == 0:
        raise ValueError('The input tag is empty!')
    tag_lst.sort()
    return tag_lst


def build_query_list(road_tag: str, infr_tag: str, man_tag: str, env_tag: str) -> List[str]:
    query_list = []
    query_road = get_single_dimension_query_list(road_tag.split(';'))
    query_infr = get_single_dimension_query_list(infr_tag.split(';'))
    query_man = get_single_dimension_query_list(man_tag.split(';'))
    query_env = get_single_dimension_query_list(env_tag.split(';'))

    query_list.extend(list(itertools.product(query_road, query_infr, query_man, query_env)))
    query_list = [';'.join(item) if not isinstance(item, str) else item for item in query_list]

    return query_list


def TagRetrieve(query: str, db: pd.DataFrame, tag2rule_dict: dict) -> List[MetaTrafficRule]:
    """
    Retrieve the rules with the same tag.
    """
    # """
    # Traverse the database to compare two sorted tag-list to get the matched rules.
    # In this way, the order of the tags in the query(and the database) is not important, since sort() function is used.
    # """
    # res = []
    # query = query.split(';')
    # query.sort()
    # for i in range(db.shape[0]):
    #     rule_tag_code = ';'.join([str(db.loc[i,ROAD_TAG_CODE_COL]),str(db.loc[i,INFR_TAG_CODE_COL]),
    #                               str(db.loc[i,MAN_TAG_CODE_COL]),str(db.loc[i,ENV_TAG_CODE_COL])])
    #     # sibling_pos = label_siblings(rule_tag_code,tree)
    #     rule_tag_code = rule_tag_code.split(';')
    #     rule_tag_code = [item for item in rule_tag_code if item]
    #     rule_tag_code.sort()
    #     if query==rule_tag_code:
    #         res.append(MetaTrafficRule(content=db.loc[i,CONTENT_COL],article=db.loc[i,ARTICLE_COL],source=db.loc[i,SOURCE_COL]))

    """
    To accelerate the retrieval process, we furthur employed a dictionary to store the rules with the same tag.
    In this way, the order of the tags in the query(and the database) is important, since all of the tags are string type in the dictionary keys.
    Without traversing the database, we can directly retrieve the rules according to the key.
    """
    res = []
    res_rowid = tag2rule_dict.get(query)
    if res_rowid:
        for rowid in res_rowid:
            res.append(
                MetaTrafficRule(
                    content=db.loc[rowid, CONTENT_COL],
                    article=db.loc[rowid, ARTICLE_COL],
                    source=db.loc[rowid, SOURCE_COL]
                )
            )
    return res


def HierarchicalRetrieve(
        res: dict, scenario_id: str, segment_id: str,
        road_tag: str, infr_tag: str, man_tag: str, env_tag: str,
        db: pd.DataFrame, tag2rule_dict: dict,
) -> Tuple[dict, List[MetaTrafficRule]]:
    """
    Utilize Tag-based Hierarchical Retrieve to find the traffic rules which apply to the given scenario.
    """
    """
    # In this method, the "query_list" occupies extremely large memory, which is not suitable for full-elements-scenarios.
    query_list = build_query_list(road_tag,infr_tag,man_tag,env_tag)
    all_retrieved_rules = []
    for query_tag in query_list:
        value = TagRetrieve(query=query_tag,db=db,tag2rule_dict=tag2rule_dict)
        if value:
        # if True:
            res[scenario_id][segment_id]['retrieved_rules'].update({
                query_tag:{
                    'number':len(value),
                    'source':[rule.source for rule in value],
                    'article':[rule.article for rule in value],
                    'content':[rule.content for rule in value]
                },
            })
            # append the retrieved MetaTrafficRule to the list
            all_retrieved_rules.extend(value)
    """

    query_road = get_single_dimension_query_list(road_tag.split(';'))
    query_infr = get_single_dimension_query_list(infr_tag.split(';'))
    query_man = get_single_dimension_query_list(man_tag.split(';'))
    query_env = get_single_dimension_query_list(env_tag.split(';'))
    all_retrieved_rules = []
    for key in list(tag2rule_dict.keys()):
        key_road = [item for item in key.split(';') if item.startswith('/root/道路')]
        key_infr = [item for item in key.split(';') if item.startswith('/root/基础设施')]
        key_man = [item for item in key.split(';') if item.startswith('/root/交通管理')]
        key_env = [item for item in key.split(';') if item.startswith('/root/环境')]
        key_road.sort()
        key_infr.sort()
        key_man.sort()
        key_env.sort()
        key_road = ';'.join(key_road) if key_road else '/root'
        key_infr = ';'.join(key_infr) if key_infr else '/root'
        key_man = ';'.join(key_man) if key_man else '/root'
        key_env = ';'.join(key_env) if key_env else '/root'
        if key_road in query_road and key_infr in query_infr and key_man in query_man and key_env in query_env:
            value = TagRetrieve(query=key, db=db, tag2rule_dict=tag2rule_dict)
            # res[scenario_id][segment_id]['retrieved_rules'].update({
            #     key:{
            #         'number':len(value),
            #         'source':[rule.source for rule in value],
            #         'article':[rule.article for rule in value],
            #         'content':[rule.content for rule in value]
            #     },
            # })
            # append the retrieved MetaTrafficRule to the list
            all_retrieved_rules.extend(value)

    return res, all_retrieved_rules

