vlib work
vlib riviera

vlib riviera/blk_mem_gen_v8_4_5
vlib riviera/xil_defaultlib

vmap blk_mem_gen_v8_4_5 riviera/blk_mem_gen_v8_4_5
vmap xil_defaultlib riviera/xil_defaultlib

vlog -work blk_mem_gen_v8_4_5  -v2k5 \
"../../../ipstatic/simulation/blk_mem_gen_v8_4.v" \

vlog -work xil_defaultlib  -v2k5 \
"../../../../exp3.gen/sources_1/ip/block_ram/sim/block_ram.v" \


vlog -work xil_defaultlib \
"glbl.v"

