import sys
exe="../debug/mahpf"
fn_folder="../bench_mark/"
fn_opt="../exp/batch2.csv"
all=[]
for m in ["empty-8-8", "empty-16-16", "random-32-32-10", "warehouse-10-20-10-2-1"]:
    fn_m= f"{fn_folder}{m}/map.map"
    for i_type in ["random","even"]:
        for i in range(1,26):
            fn_ins=f"{fn_folder}{m}/{i_type}-{i}.scen"
            with open(fn_ins) as f:
                maxlen=len(f.read().strip().split("\n"))-1

            for r in range(10,min(101,maxlen)):
                for init_method in ["OPTIMAL"]:
                    for merge_method in ["stop", "MCP","superMCP", "Sub-OPTIMAL-P1","Sub-OPTIMAL","OPTIMAL"]:
                        cmd=[exe,"-m",fn_m,"-a",fn_ins, "-r",r,"-t",60,
                             "--initAlgo",init_method,"--mergeAlgo",merge_method,
                             "--stats",fn_opt]
                        all.append(cmd)

with open(sys.argv[1],"w") as f:
    [print(*i,file=f) for i in all]



