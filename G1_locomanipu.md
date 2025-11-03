
## collect demonstrations:

```

./isaaclab.sh -p scripts/tools/record_demos.py \
--device cpu \
--task Isaac-PickPlace-Locomanipulation-G1-Abs-v0 \
--teleop_device handtracking \
--dataset_file ./datasets/locomanipu_G1/dataset_g1_locomanip.hdf5 \
--num_demos 5 --enable_pinocchio

```


## replay the collected

```
./isaaclab.sh -p scripts/tools/replay_demos.py \
--device cpu \
--task Isaac-PickPlace-Locomanipulation-G1-Abs-v0 \
--dataset_file ./datasets/locomanipu_G1/dataset_g1_locomanip.hdf5 --enable_pinocchio

```

## annotate the demonstrations

```

./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
--device cpu \
--task Isaac-Locomanipulation-G1-Abs-Mimic-v0 \
--input_file ./datasets/dataset_g1_locomanip.hdf5 \
--output_file ./datasets/dataset_annotated_g1_locomanip.hdf5 --enable_pinocchio
```



## generate a new dataset with 1000
```

./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
--device cpu --headless --num_envs 20 --generation_num_trials 1000 --enable_pinocchio \
--input_file ./datasets/dataset_annotated_g1_locomanip.hdf5 --output_file ./datasets/generated_dataset_g1_locomanip.hdf5

```



# Generate the dataset with manipulation and point-to-point navigation

##  generate the locomanipulation dataset 

- input dataset (--dataset) should be the manipulation dataset generated in the previous

```

./isaaclab.sh -p \
    scripts/imitation_learning/locomanipulation_sdg/generate_data.py \
    --device cpu \
    --kit_args="--enable isaacsim.replicator.mobility_gen" \
    --task="Isaac-G1-SteeringWheel-Locomanipulation" \
    --dataset ./datasets/generated_dataset_g1_locomanip.hdf5 \
    --num_runs 1 \
    --lift_step 60 \
    --navigate_step 130 \
    --enable_pinocchio \
    --output_file ./datasets/generated_dataset_g1_locomanipulation_sdg.hdf5 \
    --enable_cameras

```