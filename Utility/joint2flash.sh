#!/bin/bash
# Author: Minzhao
# Date: 2016-9-23
# Verison: 0.1
# This script is used to cat cpu* binary file into one flash image
# Note: if you run this script and get the following error on linux:
#       /bin/bash^M bad interpreter:No such file or directory
# you could do like this to convert the file to linux line-ending format:
#       vi joint2flash.sh
#       :set ff=unix and :wq

# function helptext()
# {
#    echo "[Usage:] "$0" -i skycpu0.txt skycpu1.txt skycpu2.txt -o flash.image"
#    exit
# }

#boot
#0->32K
#########################################################################
#build date               8byte
#version                  4byte 1byte(boot version) + 3byte(SDK version)
#upgrade block image size 4byte
#upgrade block image md5  16byte
#0->32K+256byte
#upgrade image
#0->32K+256byte+(64K-256byte)
#########################################################################
#build date               8byte
#version                  4byte 1byte(boot version) + 3byte(SDK version)
#upgrade block image size 4byte
#upgrade block image md5  16byte
#0->32K+256byte+(64K-256byte)+256byte
#upgrade image
#0->32K+256byte+(64K-256byte)+256byte+(64K-256byte)
##########################################################################
#build date               8byte
#version                  4byte 1byte(boot version) + 3byte(SDK version)
#upgrade block image size 4byte
#upgrade block image md5  16byte
#0->32K+256byte+(64K-256byte)+256byte
#upgrade image
#0->32K+256byte+(64K-256byte)+256byte+(64K-256byte)
#########################################################################
#build date               8byte
#version                  4byte 1byte(app version) + 3byte(SDK version)
#app block image size     4byte
#app block image md5      16byte
#0->32K+256byte+(64K-256byte)+256byte
#app image
#0->1M
#build date               8byte
#version                  4byte 1byte(app version) + 3byte(SDK version)
#app block image size     4byte
#app block image md5      16byte
#0->32K+256byte+(64K-256byte)+256byte
#app image
#

function dec2hex()
{
    printf "%x" $1
}
outputtxt=ar8020.bin
outputboottxt=boot.bin
outputapptxt=app.bin
outputtmp=apptmp.bin
gndoutputtmp=gndapptmp.bin
bckoutputtmp=bckapptmp.bin
outcfgbin=cfgdata.bin
outputencrypttmp=encryptapptmp.bin
bootupgrade=ar8020_bootupgrade.bin

Lib=../../Output/Staging/Lib
bootload=$Lib/ar8020_boot.bin
upgrade=$Lib/ar8020_upgrade.bin
skycpu0=$Lib/ar8020_skycpu0.bin
skycpu1=$Lib/ar8020_skycpu1.bin
skycpu2=$Lib/ar8020_skycpu2.bin

gndcpu0=$Lib/ar8020_gndcpu0.bin
gndcpu1=$Lib/ar8020_gndcpu1.bin
gndcpu2=$Lib/ar8020_gndcpu2.bin

bckcpu0=$Lib/ar8020_backupcpu0.bin
bckcpu1=$Lib/ar8020_backupcpu1.bin
bckcpu2=$Lib/ar8020_backupcpu2.bin

ve=../../Utility/imageinfo

Bin=../../Output/Staging/Bin

echo "Making the image package, please wait ..."

imagedate=`date "+%Y%m%d%H%M%S"`

#get the length of bootload/cpu0cpu1/skycpu2.txt
bootloadlength=`stat --format=%s $bootload`
upgradelength=`stat --format=%s $upgrade`
skycpu0length=`stat --format=%s $skycpu0`
skycpu1length=`stat --format=%s $skycpu1`
skycpu2length=`stat --format=%s $skycpu2`
gndcpu0length=`stat --format=%s $gndcpu0`
gndcpu1length=`stat --format=%s $gndcpu1`
gndcpu2length=`stat --format=%s $gndcpu2`
bckcpu0length=`stat --format=%s $bckcpu0`
bckcpu1length=`stat --format=%s $bckcpu1`
bckcpu2length=`stat --format=%s $bckcpu2`

#90112 = 64K ITCM2 + 0x6000 (ITCM2 extension)
skycpu2strlength=$((90112 - $skycpu2length))
gndcpu2strlength=$((90112 - $gndcpu2length))
bckcpu2strlength=$((90112 - $bckcpu2length))

cat $bootload > $outputtxt
#add "0" to the 32K offset
zerolengthboot=$((32768 - $bootloadlength))
dd if=/dev/zero of=zero.image bs=$zerolengthboot count=1
cat zero.image >> $outputtxt



#upgrade boot image
#add boot block
#add YMD
imageheadlength=0
echo -n -e \\x0 >> $bootupgrade
for i in {0..6}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$imagedate" $shiftlen 2`
        echo -n -e \\x$tmp >> $bootupgrade
done
imageheadlength=$((8 + $imageheadlength))

#boot image version
versioninfo=`sed -n 1p $ve`
versioninfo=${versioninfo##*:}
echo -n -e \\x01 >> $bootupgrade
echo -n -e \\x${versioninfo:0:2} >> $bootupgrade
echo -n -e \\x${versioninfo:3:2} >> $bootupgrade
echo -n -e \\x${versioninfo:6:2} >> $bootupgrade
imageheadlength=$((4 + $imageheadlength))

bootmode=`sed -n 2p $ve`
bootmode=${bootmode##*:}
if [[ $1 == "all" ]]; then
echo -n -e \\x${bootmode:0:2} >> $bootupgrade
elif [[ $1 == "sky" ]]; then
echo -n -e \\x02 >> $bootupgrade
elif [[ $1 == "ground" ]]; then
echo -n -e \\x03 >> $bootupgrade
fi
echo -n -e \\x${bootmode:3:2} >> $bootupgrade
imageheadlength=$((2 + $imageheadlength))

upgradeboothead=$((256+$bootloadlength))
#echo $upgradeboothead
#image length
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $upgradeboothead $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $bootupgrade
done
imageheadlength=$((4 + $imageheadlength))

#echo `md5sum $bootload | cut -d ' ' -f 1`
md5=`md5sum $bootload | cut -d ' ' -f 1`
for i in {0..15}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$md5" $shiftlen 2`
        echo -n -e \\x$tmp >> $bootupgrade
done
imageheadlength=$((16 + $imageheadlength))

zerolength=$((256 - $imageheadlength))
#add "0" to the 256byte for image head
dd if=/dev/zero of=zero.image bs=$zerolength count=1
cat zero.image >> $bootupgrade
#add bootload image
cat $bootload >> $bootupgrade







#add upgrade block
#add YMD
imageheadlength=0
echo -n -e \\x0 >> $outputtxt
for i in {0..6}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$imagedate" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputtxt
done
imageheadlength=$((8 + $imageheadlength))

#upgrade image version
versioninfo=`sed -n 1p $ve`
versioninfo=${versioninfo##*:}
echo -n -e \\x01 >> $outputtxt
echo -n -e \\x${versioninfo:0:2} >> $outputtxt
echo -n -e \\x${versioninfo:3:2} >> $outputtxt
echo -n -e \\x${versioninfo:6:2} >> $outputtxt
imageheadlength=$((4 + $imageheadlength))

bootmode=`sed -n 2p $ve`
bootmode=${bootmode##*:}
if [[ $1 == "all" ]]; then
echo -n -e \\x${bootmode:0:2} >> $outputtxt
elif [[ $1 == "sky" ]]; then
echo -n -e \\x02 >> $outputtxt
elif [[ $1 == "ground" ]]; then
echo -n -e \\x03 >> $outputtxt
fi
echo -n -e \\x${bootmode:3:2} >> $outputtxt
imageheadlength=$((2 + $imageheadlength))

upgradelengthhead=$((256+$upgradelength))
#echo $upgradelengthhead
#image length
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $upgradelengthhead $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtxt
done
imageheadlength=$((4 + $imageheadlength))

#echo `md5sum $upgrade | cut -d ' ' -f 1`
md5=`md5sum $upgrade | cut -d ' ' -f 1`
for i in {0..15}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$md5" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputtxt
done
imageheadlength=$((16 + $imageheadlength))

zerolength=$((256 - $imageheadlength))
#add "0" to the 256byte for image head
dd if=/dev/zero of=zero.image bs=$zerolength count=1
cat zero.image >> $outputtxt
#add upgrade image
cat $upgrade >> $outputtxt

#add "0" to the 160K offset
zerolength=$((65536 - $upgradelengthhead))
dd if=/dev/zero of=zero.image bs=$zerolength count=1
cat zero.image >> $outputtxt
#add upgrade back up image
dd if=$outputtxt of=backupimage  skip=32768 bs=1 count=65536
cat backupimage >> $outputtxt
rm backupimage
#add app block
#add date
echo -n -e \\x0 >> $outputtxt
for i in {0..6}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$imagedate" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputtxt
done

#SDK image version
echo -n -e \\x00 >> $outputtxt
echo -n -e \\x${versioninfo:0:2} >> $outputtxt
echo -n -e \\x${versioninfo:3:2} >> $outputtxt
echo -n -e \\x${versioninfo:6:2} >> $outputtxt

if [[ $1 == "all" ]]; then
echo -n -e \\x${bootmode:0:2} >> $outputtxt
elif [[ $1 == "sky" ]]; then
echo -n -e \\x02 >> $outputtxt
elif [[ $1 == "ground" ]]; then
echo -n -e \\x03 >> $outputtxt
fi
echo -n -e \\x${bootmode:3:2} >> $outputtxt
#12 = cpu image size(4 byte) * 3
skyapplengthhead=$((256 + 12 +$skycpu0length+$skycpu1length+$skycpu2length+$skycpu2strlength))

replenishskyapp=$((4-$skyapplengthhead%4))
skyapplengthhead=$(($skyapplengthhead+$replenishskyapp))

tmplength=`stat --format=%s $outcfgbin`
skyapplengthhead=$(($skyapplengthhead+$tmplength))

replenishskyimage=$((4-$skyapplengthhead%4))
skyapplengthhead=$(($skyapplengthhead+$replenishskyimage))


#echo $skyapplengthhead
#image length
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $skyapplengthhead $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtxt
done

#add size of skycpu0 to ar8020.bin
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $skycpu0length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtmp
done
#add skycpu0.bin to ar8020.bin
cat $skycpu0 >> $outputtmp
#add size of skycpu1 to ar8020.bin
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $skycpu1length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtmp
done
#add skycpu1.bin to ar8020.bin
cat $skycpu1 >> $outputtmp
#add size of skycpu2 to ar8020.bin
#65536: 64K ITCM2
skycpu2length=65536

for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $skycpu2length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtmp
done
#add skycpu2.bin to ar8020.bin
cat $skycpu2 >> $outputtmp
for((a=0;a<=(($replenishskyapp-1));a++))
do
        echo -n -e \\x0 >> $outputtmp
done

dd if=/dev/zero of=zero.image bs=$skycpu2strlength count=1
cat zero.image >> $outputtmp

cat $outcfgbin >> $outputtmp
md5=`md5sum $outputtmp | cut -d ' ' -f 1`
for i in {0..15}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$md5" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputtxt
done

zerolength=$((256 - $imageheadlength))
#add "0" to the 256byte for image head
dd if=/dev/zero of=zero.image bs=$zerolength count=1
cat zero.image >> $outputtxt
cat $outputtmp >> $outputtxt
for((a=0;a<=(($replenishskyimage-1));a++))
do
        echo -n -e \\x0 >> $outputtxt
done

#add app block
#add date
echo -n -e \\x0 >> $outputtxt
for i in {0..6}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$imagedate" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputtxt
done

#SDK image version
echo -n -e \\x00 >> $outputtxt
echo -n -e \\x${versioninfo:0:2} >> $outputtxt
echo -n -e \\x${versioninfo:3:2} >> $outputtxt
echo -n -e \\x${versioninfo:6:2} >> $outputtxt
if [[ $1 == "all" ]]; then
echo -n -e \\x${bootmode:0:2} >> $outputtxt
elif [[ $1 == "sky" ]]; then
echo -n -e \\x02 >> $outputtxt
elif [[ $1 == "ground" ]]; then
echo -n -e \\x03 >> $outputtxt
fi
echo -n -e \\x${bootmode:3:2} >> $outputtxt

#12 = cpu image size(4 byte) * 3
gndapplengthhead=0
gndapplengthhead=$((256 + 12 +$gndcpu0length+$gndcpu1length+$gndcpu2length+$gndcpu2strlength))
replenishgndapp=$((4-$gndapplengthhead%4))

gndapplengthhead=$(($gndapplengthhead+$replenishgndapp))

tmplength=`stat --format=%s $outcfgbin`
gndapplengthhead=$(($gndapplengthhead+$tmplength))

replenishgndimage=$((4-$gndapplengthhead%4))
gndapplengthhead=$(($gndapplengthhead+$replenishgndimage))

#echo $gndapplengthhead
#image length
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $gndapplengthhead $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtxt
done

#add size of skycpu0 to ar8020.bin
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $gndcpu0length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $gndoutputtmp
done
#add gndcpu0.bin to ar8020.bin
cat $gndcpu0 >> $gndoutputtmp
#add size of skycpu1 to ar8020.bin
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $gndcpu1length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $gndoutputtmp
done
#add gndcpu1.bin to ar8020.bin
cat $gndcpu1 >> $gndoutputtmp
#add size of skycpu2 to ar8020.bin
#65536: 64K ITCM2
gndcpu2length=65536
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $gndcpu2length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $gndoutputtmp
done
#add gndcpu2.bin to ar8020.bin
cat $gndcpu2 >> $gndoutputtmp
for((a=0;a<=(($replenishgndapp-1));a++))
do
        echo -n -e \\x0 >> $gndoutputtmp
done

dd if=/dev/zero of=zero.image bs=$gndcpu2strlength count=1
cat zero.image >> $gndoutputtmp

cat $outcfgbin >> $gndoutputtmp
md5=`md5sum $gndoutputtmp | cut -d ' ' -f 1`
for i in {0..15}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$md5" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputtxt
done

zerolength=$((256 - $imageheadlength))
#add "0" to the 256byte for image head
dd if=/dev/zero of=zero.image bs=$zerolength count=1
cat zero.image >> $outputtxt
cat $gndoutputtmp >> $outputtxt
for((a=0;a<=(($replenishgndimage-1));a++))
do
        echo -n -e \\x0 >> $outputtxt
done

applengthhead=$(($gndapplengthhead+$skyapplengthhead))

zerolength=$((3145728 - $applengthhead - 163840))

dd if=/dev/zero of=zero.image bs=1 count=$zerolength
cat zero.image >> $outputtxt

#add app back up image
#add app block
#add date
echo -n -e \\x0 >> $outputtxt
for i in {0..6}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$imagedate" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputtxt
done

#SDK image version
echo -n -e \\x00 >> $outputtxt
echo -n -e \\x${versioninfo:0:2} >> $outputtxt
echo -n -e \\x${versioninfo:3:2} >> $outputtxt
echo -n -e \\x${versioninfo:6:2} >> $outputtxt

if [[ $1 == "all" ]]; then
echo -n -e \\x${bootmode:0:2} >> $outputtxt
elif [[ $1 == "sky" ]]; then
echo -n -e \\x02 >> $outputtxt
elif [[ $1 == "ground" ]]; then
echo -n -e \\x03 >> $outputtxt
fi
echo -n -e \\x${bootmode:3:2} >> $outputtxt

bckapplengthhead=$((256 + 12 +$bckcpu0length+$bckcpu1length+$bckcpu2length+$bckcpu2strlength))
replenishbckapp=$((4-$bckapplengthhead%4))

bckapplengthhead=$(($bckapplengthhead+$replenishbckapp))

tmplength=`stat --format=%s $outcfgbin`
bckapplengthhead=$(($bckapplengthhead+$tmplength))

replenishbckimage=$((4-$bckapplengthhead%4))
bckapplengthhead=$(($bckapplengthhead+$replenishbckimage))


echo $bckapplengthhead
#image length
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $bckapplengthhead $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtxt
done

#add size of bckcpu0 to ar8020.bin
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $bckcpu0length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $bckoutputtmp
done
#add bckcpu0.bin to ar8020.bin
cat $bckcpu0 >> $bckoutputtmp
#add size of bckcpu1 to ar8020.bin
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $bckcpu1length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $bckoutputtmp
done
#add bckcpu1.bin to ar8020.bin
cat $bckcpu1 >> $bckoutputtmp
#add size of bckcpu2 to ar8020.bin
#65536: 64K ITCM2
bckcpu2length=65536

for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $bckcpu2length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $bckoutputtmp
done
#add bckcpu2.bin to ar8020.bin
cat $bckcpu2 >> $bckoutputtmp
for((a=0;a<=(($replenishbckapp-1));a++))
do
        echo -n -e \\x0 >> $bckoutputtmp
done

dd if=/dev/zero of=zero.image bs=$bckcpu2strlength count=1
cat zero.image >> $bckoutputtmp

cat $outcfgbin >> $bckoutputtmp
md5=`md5sum $bckoutputtmp | cut -d ' ' -f 1`
for i in {0..15}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$md5" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputtxt
done

zerolength=$((256 - $imageheadlength))
#add "0" to the 256byte for image head
dd if=/dev/zero of=zero.image bs=$zerolength count=1
cat zero.image >> $outputtxt
cat $bckoutputtmp >> $outputtxt
for((a=0;a<=(($replenishbckimage-1));a++))
do
        echo -n -e \\x0 >> $outputtxt
done
rm $bckoutputtmp


#cut image

if [ -f $outputapptxt ]; then
    rm $outputapptxt
fi

dd_apptmp=dd_apptmp.bin
dd if=$outputtxt of=$dd_apptmp   skip=163840 bs=1 count=$applengthhead

decryptapplengthhead=$(($applengthhead+36))
echo $decryptapplengthhead
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $decryptapplengthhead $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputapptxt
done

enmd5=`md5sum $dd_apptmp | cut -d ' ' -f 1`
for i in {0..15}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$enmd5" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputapptxt
done

enmd5=`md5sum $dd_apptmp | cut -d ' ' -f 1`
for i in {0..15}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$enmd5" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputapptxt
done

cat $dd_apptmp >> $outputapptxt

# if [ -f $outputboottxt ]; then
#     rm $outputboottxt
# fi
# dd_upgradetmp=dd_upgradetmp.bin
# dd if=$outputtxt of=$dd_upgradetmp  skip=32768 bs=1 count=$upgradelengthhead
# upgradelengthhead=$(($upgradelengthhead+36))

# echo $upgradelengthhead
# for i in {0..3}
# do
#         shiftlen=$[ i * 8 ]
#         tmp=`echo $upgradelengthhead $shiftlen | awk '{print rshift($1,$2)}'`
#         tmp=`echo $tmp | awk '{print and($1,255)}'`
#         tmphex=$(dec2hex $tmp)
#         echo -n -e \\x$tmphex >> $outputboottxt
# done

# enmd5=`md5sum $dd_upgradetmp | cut -d ' ' -f 1`
# for i in {0..15}
# do
#         shiftlen=$[ i * 2 + 1]
#         tmp=`expr substr "$enmd5" $shiftlen 2`
#         echo -n -e \\x$tmp >> $outputboottxt
# done

# enmd5=`md5sum $dd_upgradetmp | cut -d ' ' -f 1`
# for i in {0..15}
# do
#         shiftlen=$[ i * 2 + 1]
#         tmp=`expr substr "$enmd5" $shiftlen 2`
#         echo -n -e \\x$tmp >> $outputboottxt
# done

# cat $dd_upgradetmp >> $outputboottxt



if [ $# -eq 3 ]
then
    encrypt_ar8020=encrypt_ar8020.bin
    encrypt_app=encrypt_app.bin
    encrypt_apptmp=encrypt_apptmp.bin

    ../../Utility/encrypr_image $outputtxt $2 $3 $outputtxt
    dd if=$encrypt_ar8020 of=$encrypt_apptmp   skip=163840 bs=1 count=$applengthhead

    encryptapplengthhead=$(($applengthhead+36))
    echo $encryptapplengthhead
    for i in {0..3}
    do
            shiftlen=$[ i * 8 ]
            tmp=`echo $encryptapplengthhead $shiftlen | awk '{print rshift($1,$2)}'`
            tmp=`echo $tmp | awk '{print and($1,255)}'`
            tmphex=$(dec2hex $tmp)
            echo -n -e \\x$tmphex >> $encrypt_app
    done

    enmd5=`md5sum $encrypt_apptmp | cut -d ' ' -f 1`
    for i in {0..15}
    do
            shiftlen=$[ i * 2 + 1]
            tmp=`expr substr "$enmd5" $shiftlen 2`
            echo -n -e \\x$tmp >> $encrypt_app
    done
    enmd5=`md5sum $dd_apptmp | cut -d ' ' -f 1`
    for i in {0..15}
    do
            shiftlen=$[ i * 2 + 1]
            tmp=`expr substr "$enmd5" $shiftlen 2`
            echo -n -e \\x$tmp >> $encrypt_app
    done
    cat $encrypt_apptmp >> $encrypt_app
    rm $encrypt_apptmp

    # encrypt_boottmp=encrypt_boottmp.bin
    # encrypt_boot=encrypt_boot.bin
    # dd if=$encrypt_ar8020 of=$encrypt_boottmp  skip=32768 bs=1 count=$upgradelengthhead
    # upgradelengthhead=$(($upgradelengthhead+20))
    # for i in {0..3}
    # do
    #         shiftlen=$[ i * 8 ]
    #         tmp=`echo $upgradelengthhead $shiftlen | awk '{print rshift($1,$2)}'`
    #         tmp=`echo $tmp | awk '{print and($1,255)}'`
    #         tmphex=$(dec2hex $tmp)
    #         echo -n -e \\x$tmphex >> $encrypt_boot
    # done

    # enmd5=`md5sum $encrypt_boottmp | cut -d ' ' -f 1`
    # for i in {0..15}
    # do
    #         shiftlen=$[ i * 2 + 1]
    #         tmp=`expr substr "$enmd5" $shiftlen 2`
    #         echo -n -e \\x$tmp >> $encrypt_boot
    # done
    # cat $encrypt_boottmp >> $encrypt_boot
    # rm $encrypt_boottmp

fi

rm zero.image
rm $outputtmp
rm $gndoutputtmp
rm $dd_apptmp
#rm $dd_upgradetmp









