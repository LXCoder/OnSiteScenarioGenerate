
import json

class diagnose:

    def __init__(self, data):
        self.userScore = data["AbilityDimension"]
        self.diagnoseSuggestionDict = {}
        self.safeDiagnosticBase = {}
        self.efficiencyDiagnosticBase = {}
        self.comfortableDiagnosticBase = {}
        self.coordinationDiagnosticBase = {}
        self.complianceDiagnosticBase = {}
        self.safeSuggestionDiagnosticBase = {}
        self.efficiencySuggestionDiagnosticBase = {}
        self.comfortableSuggestionDiagnosticBase = {}
        self.coordinationSuggestionDiagnosticBase = {}
        self.complianceSuggestionDiagnosticBase = {}
        self.suggestionDiagnosticBase = {}

    def getSuggestion(self):
        self.diagnosticStatementBase()
        safe = self.userScore["safe"]
        efficiency = self.userScore["efficiency"]
        comfortable = self.userScore["comfortable"]
        coordination = self.userScore["coordination"]
        compliance = self.userScore["compliance"]

        Suggestion = ""
        if safe < 60:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[0]
            Suggestion += self.safeSuggestionDiagnosticBase[0]
            Suggestion += " "
        elif safe < 80:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[60]
            Suggestion += self.safeSuggestionDiagnosticBase[60]
            Suggestion += " "
        elif safe < 100:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[80]
            Suggestion += self.safeSuggestionDiagnosticBase[80]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[100]
            Suggestion += self.safeSuggestionDiagnosticBase[100]
            Suggestion += " "

        if comfortable < 60:
            self.diagnoseSuggestionDict["舒适性方面"] = self.comfortableDiagnosticBase[0]
            Suggestion += self.comfortableSuggestionDiagnosticBase[0]
            Suggestion += " "
        elif comfortable < 100:
            self.diagnoseSuggestionDict["舒适性方面"] = self.comfortableDiagnosticBase[60]
            Suggestion += self.comfortableSuggestionDiagnosticBase[60]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["舒适性方面"] = self.comfortableDiagnosticBase[100]
            Suggestion += self.comfortableSuggestionDiagnosticBase[100]
            Suggestion += " "

        if efficiency < 60:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[0]
            Suggestion += self.efficiencySuggestionDiagnosticBase[0]
            Suggestion += " "
        elif efficiency < 100:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[60]
            Suggestion += self.efficiencySuggestionDiagnosticBase[60]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[100]
            Suggestion += self.efficiencySuggestionDiagnosticBase[100]
            Suggestion += " "

        if coordination < 60:
            self.diagnoseSuggestionDict["交通协调性方面"] = self.coordinationDiagnosticBase[0]
            Suggestion += self.coordinationSuggestionDiagnosticBase[0]
            Suggestion += " "
        elif coordination < 100:
            self.diagnoseSuggestionDict["交通协调性方面"] = self.coordinationDiagnosticBase[60]
            Suggestion += self.coordinationSuggestionDiagnosticBase[60]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["交通协调性方面"] = self.coordinationDiagnosticBase[100]
            Suggestion += self.coordinationSuggestionDiagnosticBase[100]
            Suggestion += " "

        if compliance < 60:
            self.diagnoseSuggestionDict["交规符合性方面"] = self.complianceDiagnosticBase[0]
            Suggestion += self.complianceSuggestionDiagnosticBase[0]
            Suggestion += " "
        elif compliance < 100:
            self.diagnoseSuggestionDict["交规符合性方面"] = self.complianceDiagnosticBase[60]
            Suggestion += self.complianceSuggestionDiagnosticBase[60]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["交规符合性方面"] = self.complianceDiagnosticBase[100]
            Suggestion += self.complianceSuggestionDiagnosticBase[100]
            Suggestion += " "

        self.diagnoseSuggestionDict["建议"] = Suggestion

    # 后续更新诊断语句，就在这里更新
    def diagnosticStatementBase(self):
        self.safeDiagnosticBase = {
            0: "安全性差，存在较严重违规行为，不能保证行车安全。",
            60: "安全性中等，存在严重违规行为，不能保证行车安全。",
            80: "安全性良好，存在较轻违规行为，不能保证行车安全。",
            100: "安全性好，不存在违规行为。"
        }
        self.efficiencyDiagnosticBase = {
            0: "任务耗时较长、效率较低，或任务未完成。",
            60: "任务耗时长，效率较低。",
            100: "任务完成，且效率较高。"
        }
        self.comfortableDiagnosticBase = {
            0: "舒适性差，存在较多急加速/急减速/急转向等行为。",
            60: "舒适性中等，存在急加速/急减速/急转向等行为。",
            100: "舒适性好，不存在急加速/急减速/急转向等行为。"
        }
        self.coordinationDiagnosticBase = {
            0: "交通协调性差，存在较多影响其他车辆通行的行为。",
            60: "交通协调性中等，存在影响其他车辆通行的行为。",
            100: "交通协调性好，不存在影响其他车辆通行的行为。"
        }
        self.complianceDiagnosticBase = {
            0: "交规符合性差，存在较多违规行为。",
            60: "交规符合性中等，存在较多违规行为。",
            100: "交规符合性好，不存在较多违规行为。"
        }
        self.safeSuggestionDiagnosticBase = {
            0: "从安全性方面，有必要提升控制规划算法，提升交通安全。",
            60: "从安全性方面，有必要提升控制规划算法，提升交通安全。",
            80: "从安全性方面，建议提升控制规划算法，提升交通安全。",
            100: "从安全性方面，安全性很好，无提升建议。"
        }
        self.efficiencySuggestionDiagnosticBase = {
            0: "从效率性方面，有必要提升控制算法，提升行驶效率。",
            60: "从效率性方面，建议提升控制算法，提升行驶效率。",
            100: "从效率性方面，算法很高效，无提升建议。"
        }
        self.comfortableSuggestionDiagnosticBase = {
            0: "从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。",
            60: "从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。",
            100: "从舒适性方面，舒适性好，无提升建议。"
        }
        self.coordinationSuggestionDiagnosticBase = {
            0: "从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。",
            60: "从交通协调性方面，建议提升控制算法和驾驶策略，提升协作效率。",
            100: "从交通协调性方面，交通协调性好，无提升建议。"
        }
        self.complianceSuggestionDiagnosticBase = {
            0: "从交规符合性方面，有必要提升控制规划算法，减少违规次数。",
            60: "从交规符合性方面，建议提升控制规划算法，减少违规次数。",
            100: "从交规符合性方面，交规符合性好，无提升建议。"
        }

        self.suggestionDiagnosticBase = {
            "safe": "从安全性角度出发，建议提升路径规划算法，提升决策规划算法，提升速度规划算法及车辆控制算法。",
            "efficiency": "从效率角度出发，建议提升驾驶策略，增加域控制器算法的激进程度，主动与其他车辆进行交互。",
            "comfortable": "从舒适性角度出发，建议提升速度规划算法及车辆控制算法，尽可能平滑的转向，减少反复修正转向角的次数。",
            "wellDone": "在当前测试场景下，从安全性、舒适性、效率评价，成绩优异，建议更换场景测试，或选择更多维度再次评价。"
        }
