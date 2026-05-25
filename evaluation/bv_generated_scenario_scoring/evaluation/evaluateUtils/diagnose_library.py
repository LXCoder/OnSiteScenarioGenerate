
import json


class diagnose:

    def __init__(self, data):
        self.userScore = data["AbilityDimension"]
        self.diagnoseSuggestionDict = {}
        self.safeDiagnosticBase = {}
        self.efficiencyDiagnosticBase = {}
        self.comfortableDiagnosticBase = {}
        self.suggestionDiagnosticBase = {}

    def getSuggestion(self):
        self.diagnosticStatementBase()
        safe = self.userScore["safe"]
        efficiency = self.userScore["efficiency"]
        comfortable = self.userScore["comfortable"]
        safeSuggestion = 1
        efficiencySuggestion = 1
        comfortableSuggestion = 1

        if safe < 80:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[80]
        elif safe < 95:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[95]
        else:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[100]
            safeSuggestion = 0

        if comfortable < 70:
            self.diagnoseSuggestionDict["舒适性方面"] = self.comfortableDiagnosticBase[70]
        elif comfortable < 90:
            self.diagnoseSuggestionDict["舒适性方面"] = self.comfortableDiagnosticBase[90]
        else:
            self.diagnoseSuggestionDict["舒适性方面"] = self.comfortableDiagnosticBase[100]
            comfortableSuggestion = 0

        if efficiency < 50:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[50]
        elif efficiency < 70:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[70]
        elif efficiency < 90:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[90]
        else:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[100]
            efficiencySuggestion = 0

        Suggestion = ""
        if safeSuggestion:
            Suggestion += self.suggestionDiagnosticBase["safe"]
            Suggestion += " "
        if comfortableSuggestion:
            Suggestion += self.suggestionDiagnosticBase["comfortable"]
            Suggestion += " "
        if efficiencySuggestion:
            Suggestion += self.suggestionDiagnosticBase["efficiency"]
            Suggestion += " "
        if not safeSuggestion and not comfortableSuggestion and not efficiencySuggestion:
            Suggestion = self.suggestionDiagnosticBase["wellDone"]

        self.diagnoseSuggestionDict["建议"] = Suggestion

    # 后续更新诊断语句，就在这里更新
    def diagnosticStatementBase(self):
        self.safeDiagnosticBase = {
            80 : "安全性方面，成绩不合格，存在严重违规行为，不可保证行车安全。",
            95 : "安全性方面，成绩中等，存在部分违规行为，可基本保证行车安全。",
            100 : "安全性方面，取得良好成绩，符合驾驶规范，可保证行车安全。"
        }
        self.efficiencyDiagnosticBase = {
            50: "交互决策过于保守，拟人程度过低，导致通过场景时间过长，效率指标不合格。",
            70: "交互决策保守，拟人程度低，导致通过场景时间较长，效率指标中等。",
            90: "交互决策保守，拟人程度中等，通过场景时间正常，效率指标良。",
            100: "交互决策激进，拟人程度高，通过场景时间较短，效率指标优异。"
        }
        self.comfortableDiagnosticBase = {
            70: "舒适性方面，成绩不合格，存在严重不良驾驶行为，乘坐舒适度差。",
            90: "舒适性方面，成绩中等，存在部分不良驾驶行为，可基本保证乘坐舒适度。",
            100: "舒适性方面，取得良好成绩，无不良驾驶行为，乘坐舒适度较高。"
        }
        self.suggestionDiagnosticBase = {
            "safe": "从安全性角度出发，建议提升路径规划算法，提升决策规划算法，提升速度规划算法及车辆控制算法。",
            "efficiency": "从效率角度出发，建议提升驾驶策略，增加域控制器算法的激进程度，主动与其他车辆进行交互。",
            "comfortable": "从舒适性角度出发，建议提升速度规划算法及车辆控制算法，尽可能平滑的转向，减少反复修正转向角的次数。",
            "wellDone": "在当前测试场景下，从安全性、舒适性、效率评价，成绩优异，建议更换场景测试，或选择更多维度再次评价。"
        }
