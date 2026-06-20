
#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>

/* タスク優先度 */
#define MAIN_PRIORITY    5 /* メインタスク */
#define TRACER_PRIORITY       6 /* ライントレーサータスク */
#define CALIBRATION_PRIORITY  5 /* キャリブレーションタスク */

/* タスク周期の定義 */
#define TRACER_PERIOD  (8 * 1000) /* トラッキングタスク:8msec周期 */


#ifndef STACK_SIZE
#define STACK_SIZE      (4096)
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void tracer_task(intptr_t exinf);
extern void calibration_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
