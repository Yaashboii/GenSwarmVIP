from enum import Enum
from modules.stages.stage import StageType

class TestResult(Enum):
    PASS = 1
    NOT_PASS= 2

class BugSource(Enum):
    CODE = 1
    TEST_CODE = 2

StageTransition = {
    StageType.AnalyzeStage: StageType.DesignStage,
    StageType.DesignStage: StageType.WriteCoreStage,
    StageType.WriteCoreStage: StageType.TestCoreStage,
    StageType.TestCoreStage: {
        TestResult.PASS: StageType.WriteMainStage,
        TestResult.NOT_PASS: {
            BugSource.CODE: StageType.WriteCoreStage,
            BugSource.TEST_CODE: StageType.TestCoreStage,
        }
    },
    StageType.WriteMainStage: StageType.TestMainStage,
    StageType.TestMainStage: {
        TestResult.PASS: StageType.FinalStage,
        TestResult.NOT_PASS: {
            BugSource.CODE: StageType.WriteMainStage,
            BugSource.TEST_CODE: StageType.TestMainStage,
        }
    }
}

if __name__ == "__main__":
    keys = [TestResult.NOT_PASS, BugSource.CODE]
    temp = StageTransition[StageType.TestCoreStage]
    for key in keys:
        temp = temp[key]
        print(temp)