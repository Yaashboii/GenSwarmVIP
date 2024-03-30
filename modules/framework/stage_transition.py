from modules.stages.stage import StageType
from modules.utils.common import TestResult, BugSource, DesignPattern, CodeMode

# StageTransition = {
#     StageType.AnalyzeStage: StageType.DesignStage,
#     StageType.DesignStage: StageType.CodingStage,
#     StageType.CodingStage: StageType.TestingStage,
#     StageType.TestingStage: {
#         TestResult.HALF_PASS: StageType.CodingStage,
#         TestResult.ALL_PASS: StageType.FinalStage,
#         TestResult.NOT_PASS: {
#             BugSource.CODE: StageType.CodingStage,
#             BugSource.TEST_CODE: StageType.TestingStage,
#         }
#     }
# }

StageTransition = {
    StageType.AnalyzeStage: StageType.CodingStage,
    # StageType.DesignStage: {
    #     DesignPattern.FUNCTION: StageType.CodingStage,
    #     DesignPattern.SEQ_DIAGRAM: StageType.CodingStage,
    # },
    StageType.CodingStage: StageType.ReviewStage,
    StageType.ReviewStage: StageType.RunningStage,
    StageType.RunningStage: {
        TestResult.ALL_PASS: StageType.FinalStage,
        TestResult.NOT_PASS: StageType.RunningStage,
    }

}

if __name__ == "__main__":
    keys = [DesignPattern.FUNCTION]
    temp = StageTransition[StageType.DesignStage]
    for key in keys:
        temp = temp[key]
    print(temp)
