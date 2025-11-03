
## Collect  demonstrations

```
./isaaclab.sh -p scripts/tools/record_demos.py \
--device cpu \
--task Isaac-PickPlace-G1-InspireFTP-Abs-v0 \
--teleop_device handtracking \
--dataset_file ./datasets/manipu_G1/dataset_g1_1_init.hdf5 --num_demos 5 \
--enable_pinocchio
```

## replay collection

```
./isaaclab.sh -p scripts/tools/replay_demos.py \
--device cpu \
--task Isaac-PickPlace-G1-InspireFTP-Abs-v0 \
--dataset_file ./datasets/manipu_G1/dataset_g1.hdf5 \
--enable_pinocchio
```

## annotate  demonstrations （mimic）

```
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
--device cpu \
--task Isaac-PickPlace-G1-InspireFTP-Abs-Mimic-v0 \
--input_file ./datasets/manipu_G1/dataset_g1.hdf5 \
--output_file ./datasets/manipu_G1/dataset_annotated_g1.hdf5 \
--enable_pinocchio
```

## Generate the dataset

```
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
--device cpu \
--headless --num_envs 20 \
--generation_num_trials 1000 \
--enable_pinocchio \
--input_file ./datasets/manipu_G1/dataset_annotated_g1.hdf5 \
--output_file ./datasets/manipu_G1/generated_dataset_g1.hdf5

```

## Train a policy

```
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
--task Isaac-PickPlace-G1-InspireFTP-Abs-v0 \
--algo bc \
--normalize_training_actions \
--dataset ./datasets/manipu_G1/generated_dataset_g1.hdf5

```

## Play a policy

```
./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
--device cpu \
--enable_pinocchio \
--task Isaac-PickPlace-G1-InspireFTP-Abs-v0 \
--num_rollouts 50 \
--horizon 400 \
--norm_factor_min 10 \
--norm_factor_max 30 \
--checkpoint logs/robomimic/Isaac-PickPlace-G1-InspireFTP-Abs-v0/bc_rnn_low_dim_g1/20251031085115/models/model_epoch_2000.pth

```