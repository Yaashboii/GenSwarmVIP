from modules.stages.stage import StageType
from modules.utils.common import TestResult, BugSource


StageTransition = {
    StageType.AnalyzeStage: StageType.DesignStage,
    StageType.DesignStage: StageType.CodingStage,
    StageType.CodingStage: StageType.TestingStage,
    StageType.TestingStage: {
        TestResult.PASS: StageType.CodingStage,
        TestResult.NOT_PASS: {
            BugSource.CODE: StageType.CodingStage,
            BugSource.TEST_CODE: StageType.TestingStage,
        }
    }
}

if __name__ == "__main__":
    keys = [TestResult.NOT_PASS, BugSource.CODE]
    temp = StageTransition[StageType.TestingStage]
    for key in keys:
        temp = temp[key]
        print(temp)
