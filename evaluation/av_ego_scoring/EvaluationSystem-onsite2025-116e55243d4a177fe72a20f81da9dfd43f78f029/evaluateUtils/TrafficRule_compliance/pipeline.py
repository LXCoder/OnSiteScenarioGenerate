import pandas as pd
from tqdm import tqdm
import json
from evaluateUtils.TrafficRule_compliance.retrieval import HierarchicalRetrieve
from evaluateUtils.config import CONTENT_COL, ROAD_TAG_CODE_COL, INFR_TAG_CODE_COL, MAN_TAG_CODE_COL, ENV_TAG_CODE_COL, SCENARIO_ID_COL, SEGMENT_ID_COL
from evaluateUtils.TrafficRule_compliance.file_utils import remove_lst_duplicates_with_order

def run_rulelist_pipeline(
        db: pd.DataFrame,
        tag2rule_dict: dict,
        scn_tag: pd.DataFrame,
        # output_path: str,
        # language: str = 'zh',
        use_rule2behavior_gt: bool = True,
):
    """
    Given the scenario tags and the traffic rules database, construct the rule list.
    The rule list will be generated in two steps.
    In the first step, a hierarchical retrieval is conducted to match the relative rules with the scenario tags.
    After that, the retrieved rules will be further splitted into two parts, prohibition and command. This step has been mannually done and stored.
    """
    """
    The rule list, along with its retrieved rules, will be saved as an json file. The file format is as below.
    {
        scenario_id:{
            segment_id:{
                "road_tag":"{the code of road tag}",
                "infr_tag":"{the code of infrastracture tag}",
                "man_tag":"{the code of management tag}",
                "env_tag":{the code of environment tag}",
                "total_number": the number of the total of retrieved rules,
                "(source,article)":[(kb1,x),(kb2,y),...],
                "rules":[rule1,rule2,...],
                "prohibitions":[CANNOT1,CANNOT2,...],
                "commands":[MUST1,MUST2,...]
            }
        }
    }
    """
    if (not use_rule2behavior_gt):
        raise ValueError('use_rule2behavior_gt must be True.')

    # define results file
    res = {}

    # start to build the rule list one by one
    for i in tqdm(range(scn_tag.shape[0])):
        # initialize the basic id and tag information to record the results
        scenario_id, segment_id = scn_tag[SCENARIO_ID_COL][i], scn_tag[SEGMENT_ID_COL][i]
        if scenario_id not in res.keys():
            res[scenario_id] = {}
        if segment_id not in res[scenario_id].keys():
            res[scenario_id][segment_id] = {}
        road_tag, infr_tag, man_tag, env_tag = \
            scn_tag[ROAD_TAG_CODE_COL][i], scn_tag[INFR_TAG_CODE_COL][i], scn_tag[MAN_TAG_CODE_COL][i], \
            scn_tag[ENV_TAG_CODE_COL][i]
        res[scenario_id][segment_id].update({
            'road_tag_code': road_tag,
            'infr_tag_code': infr_tag,
            'man_tag_code': man_tag,
            'env_tag_code': env_tag
        })
        # res[scenario_id][segment_id]['retrieved_rules'] = {}
        res, all_retrieved_rules = HierarchicalRetrieve(
            res=res, scenario_id=scenario_id, segment_id=segment_id,
            road_tag=road_tag, infr_tag=infr_tag, man_tag=man_tag, env_tag=env_tag,
            db=db, tag2rule_dict=tag2rule_dict,
        )

        # step 2: split the retrieved rules into prohibitions and commands
        res[scenario_id][segment_id]['total_number'] = len(all_retrieved_rules)
        res[scenario_id][segment_id]['(source,articles)'] = [(r.source, r.article) for r in all_retrieved_rules]
        res[scenario_id][segment_id]['rules'] = [r.content for r in all_retrieved_rules]

        cmd_list, prh_list = [], []
        # When the ground truth of the behavior is available, there is no need to use the LLM model
        if use_rule2behavior_gt:
            for rule in all_retrieved_rules:
                cmd = db.loc[db[CONTENT_COL] == rule.content, 'commands'].values[0]
                prh = db.loc[db[CONTENT_COL] == rule.content, 'prohibitions'].values[0]
                cmd_list.extend(cmd.split(';'))
                prh_list.extend(prh.split(';'))
            prh_list = [item for item in prh_list if item]
            cmd_list = [item for item in cmd_list if item]

        # drop duplicate commands and prohibitions
        prh_list = remove_lst_duplicates_with_order(prh_list)
        cmd_list = remove_lst_duplicates_with_order(cmd_list)
        # save the prohibitions and commands
        res[scenario_id][segment_id]['prohibitions'] = prh_list
        res[scenario_id][segment_id]['commands'] = cmd_list

    # save json results
    # wth:取消文件
    # with open(output_path, encoding='utf-8', mode='w') as f:
    #     json.dump(res, f, ensure_ascii=False, indent=4)

    return res


