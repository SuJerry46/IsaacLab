## docker pull nvcr.io/nvidia/cloudxr-runtime:5.0.1
```

docker run -d \
  --gpus all \
  --network host \
  --name cloudxr-runtime \
  -e "ACCEPT_EULA=Y" \
  nvcr.io/nvidia/cloudxr-runtime:5.0.1



```


## docker pull nvcr.io/nvidia/isaac-lab:2.3.0

```
docker run -d \
  --gpus all \
  --network host \
  -e "ACCEPT_EULA=Y" \
  -v ~/Jerry/IsaacLab:/workspace/isaaclab \
  --name isaac-lab-2.3 \
  nvcr.io/nvidia/isaac-lab:2.3.0 \
  tail -f /dev/null

docker exec -it isaac-lab-2.3 bash

```


## docker stop 
## docker  rm


## Use docker compose to run the Isaaclab and cloudXR Runtime containers together
```
./docker/container.py start \
    --files docker-compose.cloudxr-runtime.patch.yaml \
    --env-file .env.cloudxr-runtime
```


## Enter the Isaac Lab base container

```
./docker/container.py enter base

```

## Collect  demonstrations

### Basic version (without waist)
```
./isaaclab.sh -p scripts/tools/record_demos.py \
--device cpu \
--task Isaac-PickPlace-G1-InspireFTP-Abs-v0 \
--teleop_device handtracking \
--dataset_file ./datasets/dataset_g1_1_init.hdf5 --num_demos 5 \
--enable_pinocchio
```

### Waist enabled version
```
./isaaclab.sh -p scripts/tools/record_demos.py \
--device cpu \
--task Isaac-PickPlace-G1-InspireFTP-WaistEnabled-Abs-v0 \
--teleop_device handtracking \
--dataset_file ./datasets/dataset_g1.hdf5 --num_demos 5 \
--enable_pinocchio
```
## replay collection

```
./isaaclab.sh -p scripts/tools/replay_demos.py \
--device cpu \
--task Isaac-PickPlace-G1-InspireFTP-Abs-v0 \
--dataset_file ./datasets/dataset_g1.hdf5 \
--enable_pinocchio
```

```
./isaaclab.sh -p scripts/tools/replay_demos.py \
--device cpu \
--task Isaac-PickPlace-G1-InspireFTP-WaistEnabled-Abs-v0 \
--dataset_file ./datasets/dataset_g1.hdf5 \
--enable_pinocchio
```
## annotate  demonstrations （mimic）

```
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
--device cpu \
--task Isaac-PickPlace-G1-InspireFTP-Abs-Mimic-v0 \
--input_file ./datasets/dataset_g1.hdf5 \
--output_file ./datasets/dataset_annotated_g1.hdf5 \
--enable_pinocchio
```

```
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
--device cpu \
--task Isaac-PickPlace-G1-InspireFTP-WaistEnabled-Abs-Mimic-v0 \
--input_file ./datasets/dataset_g1.hdf5 \
--output_file ./datasets/dataset_annotated_g1.hdf5 \
--enable_pinocchio
```
### keyboard commands

```
Annotating episode #0 (demo_0)
   Playing the episode for subtask annotations for eef "right".
   Subtask signals to annotate:
      - Termination:      ['idle_right']

   Press "N" to begin.
   Press "B" to pause.
   Press "S" to annotate subtask signals.
   Press "Q" to skip the episode.
```
## Generate the dataset

```
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
--device cpu \
--headless --num_envs 20 \
--generation_num_trials 1000 \
--enable_pinocchio \
--input_file ./datasets/dataset_annotated_g1.hdf5 \
--output_file ./datasets/generated_dataset_g1.hdf5

```

## Train a policy

```
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
--task Isaac-PickPlace-G1-InspireFTP-Abs-v0 \
--algo bc \
--normalize_training_actions \
--dataset ./datasets/generated_dataset_g1.hdf5

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
--checkpoint logs/robomimic/Isaac-PickPlace-G1-InspireFTP-Abs-v0/bc_rnn_low_dim_gr1t2/20251031085115/models/model_epoch_2000.pth

```