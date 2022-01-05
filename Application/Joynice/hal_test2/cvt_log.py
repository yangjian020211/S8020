import sys
import struct

MSG_ID = b'\x02'


def print_bytes_hex(bytes_array):
    text = ''
    for x in bytes_array:
        text += '0x%X,' % x
    print(text)


def find_wrap_by(barray, starter, ender=None):
    if ender is None:
        ender = starter
    out_array = []
    idx_end = 0
    while True:
        idx_start = barray.find(starter, idx_end)
        if idx_start >= idx_end:
            idx_start += len(starter)
            idx_end = barray.find(ender, idx_start)
            if idx_end > idx_start:
                out_array.append(barray[idx_start:idx_end].strip())
            else:
                break
        else:
            break
    return out_array


'''
使用 python cvt_log.py 源文件 目标文件
'''
if __name__ == '__main__':

    # 读取命令行参数
    bin_file_name = './datafile_00000002.log'
    out_file_name = './out.txt'
    if len(sys.argv) == 2:
        bin_file_name = sys.argv[1]
    elif len(sys.argv) == 3:
        bin_file_name = sys.argv[1]
        out_file_name = sys.argv[2]

    # 读取所有数据到
    fmt2 = b'\xa3\x95'
    fmt1 = fmt2 + MSG_ID
    file = open(bin_file_name, 'rb')
    content = file.read()
    data_array = find_wrap_by(content, fmt1, fmt2)
    file.close()

    print(len(data_array))

    # 格式化数据 并 统计格式错误的数据项目
    s = struct.Struct('I12f')
    txt = ''
    data_broken_count = 0.0
    for index in range(len(data_array)):
        data = data_array[index]
        if len(data) != 52:
            print('data broken at index %d' % index)
            data_broken_count += 1
            continue
            # data += (b'\0' * (52 - len(data)))
        values = s.unpack_from(data)
        for v in values[1:]:
            txt += '%f, ' % v
        txt += '\n'

    # 统计出错结果
    broken_pct = data_broken_count / len(data_array)
    broken_pct *= 100.0
    print('total broken %d @%f%%' % (data_broken_count, broken_pct))

    # 将转换结果输出到文件
    fout = open(out_file_name, 'w')
    fout.write(txt)
    fout.close()
