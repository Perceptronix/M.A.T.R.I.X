# MEMBER3 model contract

## Model

PIDNet-S, trained for RELLIS-3D with 19 semantic classes. The deployment graph
uses the inference form with `augment=False` and loads the existing
`best.pt`; it does not retrain or alter weights.

## RELLIS mapping

| Training ID | Class | Raw RELLIS IDs |
| ---: | --- | --- |
| 0 | void | 0, 1 |
| 1 | grass | 3, 29, 30 |
| 2 | mud | 4 |
| 3 | bush | 5 |
| 4 | concrete | 6, 32 |
| 5 | sky | 7 |
| 6 | water | 8 |
| 7 | puddle | 9 |
| 8 | dirt | 10 |
| 9 | gravel | 12 |
| 10 | asphalt | 15 |
| 11 | building | 17 |
| 12 | log | 18 |
| 13 | person | 19 |
| 14 | fence | 23 |
| 15 | vehicle | 27 |
| 16 | object | 31 |
| 17 | pole | 33 |
| 18 | tree trunk | 34 |

Unlisted raw label IDs map to the dataset ignore label (`255`) during dataset
label conversion. Ignore labels are not an ONNX output class.

## Postprocessing

For logits `z`, compute `p = softmax(z, axis=class)`. The class map is
`argmax(p, axis=class)` and confidence is `max(p, axis=class)`.

The graph input is `(1, 3, 512, 1024)` `float32`; the graph output is
`(1, 19, 64, 128)` `float32`.