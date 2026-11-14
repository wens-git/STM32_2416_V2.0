/**
 * @file    input.c
 * @brief   通用数字输入消抖与边沿事件检测实现。
 * @details 每个 Input_t 对象通过 read_pin 回调获取逻辑输入状态。连续采样达到
 *          filter_max 后才更新稳定状态，并锁存按下或释放事件。事件采用读取后
 *          清除语义，稳定状态和事件标志可由主循环安全查询。
 */

#include "input.h"

/**
 * @brief  初始化输入按键对象
 * @note   该函数用于配置按键获取状态的函数指针以及消抖时间，并将状态机的各项参数复位到初始状态。
 * @param  in: 指向待初始化的按键输入结构体对象的指针
 * @param  read_pin: 能够获取引脚电平的函数指针 (例如 IODEF.c 里的 X0)
 * @param  debounce_ticks: 按键消抖阈值。在定时器扫描中，需要连续采样到相同电平达到此次数，才认为状态稳定。
 * @retval 无
 */
void input_init(Input_t *in, uint8_t (*read_pin)(void), uint16_t debounce_ticks)
{
    /* 保存硬件接口读取函数 */
    in->read_pin = read_pin;

    /* 初始化消抖滤波器参数 */
    in->filter_cnt = 0;              // 当前连续一致采样次数清零
    in->filter_max = debounce_ticks; // 设置触发状态改变所需的连续采样阈值
    
    /* 初始化状态机运行标志位 */
    in->raw_last = false;            // 上一次采样的原始电平初始默认设为 false
    in->stable = false;              // 当前认为的稳定电平状态，初始默认设为 false
    in->is_initialized = false;      // 标记首次采样尚未进行，用于开机第一帧状态同步
    
    /* 清除所有挂起的事件标志 */
    in->evt_down = false;            // 清除按下事件（逻辑上升沿）标志
    in->evt_up = false;              // 清除释放事件（逻辑下降沿）标志
}

/**
 * @brief  更新按键输入状态（状态机核心驱动函数）
 * @note   该函数必须在定时器中断或周期性任务中以固定频率（如 1ms 或 10ms）被持续调用。
 *         它负责读取 GPIO 物理电平，进行软件滤波（积分式消抖），并在状态发生确定改变时抛出按下/释放事件。
 * @param  in: 指向按键输入结构体对象的指针
 * @retval 无
 */
void input_update(Input_t *in)
{
    /* 安全检查：如果未绑定读取函数，则直接返回，避免引发硬错误 */
    if (in->read_pin == NULL) return;

    /*
     * 第一步：读取当前输入逻辑态
     * raw = true  表示输入逻辑为 True (触发/按下)
     * raw = false 表示输入逻辑为 False (未触发/释放)
     */
    bool raw = (in->read_pin() == 1);

    /* 
     * 第二步：开机第一帧特殊处理
     * 首次运行此函数时，不进行消抖计算，直接将当前读取到的电平视为稳定状态。
     * 这样可以防止开机时按键默认状态与代码初始状态不一致导致的误触发。
     */
    if (!in->is_initialized)
    {
        in->stable = raw;            // 将当前物理电平强制作为初始稳定电平
        in->raw_last = raw;          // 同步上次采样值
        in->is_initialized = true;   // 标记已完成首次初始化
        return;                      // 第一帧处理完毕，直接返回
    }

    /* 
     * 第三步：消抖滤波逻辑 - 采样值发生跳变
     * 如果当前读取的电平与上一次读取的不一样（说明存在抖动，或者按键刚刚被按下/松开），
     * 必须立即重置连续一致计数器，重新开始计数，并更新上一次的采样值为当前值。
     */
    if (raw != in->raw_last)
    {
        in->raw_last = raw;          // 更新上次采样值
        in->filter_cnt = 0;          // 计数器清零，重新开始"积分"
        return;                      // 跳变期间不进行后续状态判断，直接返回
    }

    /* 
     * 第四步：消抖滤波逻辑 - 采样值保持一致
     * 如果当前读取的电平和上一次一样，说明电平保持住了。
     * 此时让连续一致计数器累加。如果还没达到我们设定的消抖阈值（filter_max），
     * 就继续等待下一次周期调用。
     */
    if (in->filter_cnt < in->filter_max)
    {
        in->filter_cnt++;            // 连续相同电平计数加 1
        return;                      // 尚未达到稳定条件，直接返回
    }

    /* 
     * 第五步：状态确认与事件触发
     * 走到这里，说明电平不仅没有跳变，而且连续一致的次数已经达到了设定的消抖阈值。
     * 这说明当前的按键电平是真实、稳定的，而不是抖动干扰。
     * 此时我们需要检查这个新确认的稳定状态是否与系统之前记录的稳定状态不同。
     */
    if (in->stable != raw)
    {
        in->stable = raw;            // 将系统记录的稳定状态更新为当前的新状态

        /* 逻辑态约定：True=按下/触发，False=释放/未触发 */
        if (raw)
            in->evt_down = true;     // False -> True：按下事件
        else
            in->evt_up = true;       // True  -> False：释放事件
    }
}

/**
 * @brief  获取按键当前的实时稳定状态
 * @note   获取的是经过消抖处理后的稳定逻辑状态，而非 GPIO 瞬间的物理电平。
 * @param  in: 指向按键输入结构体对象的指针
 * @retval true: 输入当前正处于按下/触发状态
 * @retval false: 输入当前正处于释放/未触发状态
 */
bool input_get(Input_t *in)
{
    return in->stable;
}

/**
 * @brief  检查按键是否刚刚发生过"按下"动作 (读取后自动清除标志)
 * @note   用于事件驱动模式。如果在两次调用此函数之间按键被按下过，将返回 true。
 *         为了防止在读取和清除标志位时被中断打断导致状态混乱，使用了原子操作保护。
 * @param  in: 指向按键输入结构体对象的指针
 * @retval true: 检测到按下事件
 * @retval false: 没有按下事件
 */
bool input_check_down(Input_t *in)
{
    bool ret = false;
    ATOMIC_ENTER();                  // 进入临界区，关闭全局中断或调度器，防止事件标志被并发修改
    if (in->evt_down)
    {
        in->evt_down = false;        // 读取到事件后，立即清除标志位，防止重复处理（"读后清零"机制）
        ret = true;                  // 记录发生了事件
    }
    ATOMIC_EXIT();                   // 退出临界区，恢复中断或调度器
    return ret;
}

/**
 * @brief  检查按键是否刚刚发生过"释放"动作 (读取后自动清除标志)
 * @note   同 input_check_down，用于检测按键弹起的瞬间动作。
 * @param  in: 指向按键输入结构体对象的指针
 * @retval true: 检测到释放事件
 * @retval false: 没有释放事件
 */
bool input_check_up(Input_t *in)
{
    bool ret = false;
    ATOMIC_ENTER();                  // 进入临界区，保护标志位操作
    if (in->evt_up)
    {
        in->evt_up = false;          // 读后清零释放标志位
        ret = true;                  // 记录发生了释放事件
    }
    ATOMIC_EXIT();                   // 退出临界区
    return ret;
}
