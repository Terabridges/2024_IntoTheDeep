package Autonomous;

public class AutoPather {
    /*
    Idea: AutoPather
    Create two paths that loop, so 1 -> 2 -> 1 -> 2... etc
    Create method changePath that adds amount to path not currently executing
    then call buildPaths again between changing paths
    Basically, updates paths directly after changing between the two looping
    in order to clear up having a million paths

    Add a voltage modifier that detects voltage in init and averages or finds constant
    (as voltage can fluctuate) and adds modifier to path generator to adjust for voltage
    This will help auto be consistent and lessen worry about have perfectly charged batteries

    Make sure tuning is perfect, make sure robot detects expected position rather than
    having weird offset
    */
}
