#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <vector>
#include <algorithm>

// bu kod ROS ile turtlebot3'un mouse maze labirentini cozmesini sagliyor. sag duvari takip ediyoruz. uzun sure calistirdiktan sonra sonuca ulasmayi basardim. hedefe ulasabiliyor. bazi yerlerde zorlaniyor ancak yine de cozmeyi basariyor. bazen koselerde sikisiyormus gibi gozukuyor. bunu cozmek icin time interval da ekledim, bu sekilde sikismiyor ancak koseleri yavas donuyor. bu kodu daha iyi hale getirmek icin bazi parametreleri degistirdim. bu parametreler robotun duvara olan mesafesini, toleransini ve kose durumunu kontrol ediyor. ayrica robotun odometrisini takip ediyoruz ve hedefe ulastiginda durmasini sagliyoruz. diger kodumdan farklı olan şey burada parelellik sorununu biraz daha çözdüm salınım yaparak duvara biraz daha uzaktan ilerleyebiliyor.
ros::Publisher cmd_pub;
bool hedefe_ulasildi = false;
enum Durum
{
    DUVAR_ARA,
    DUVARA_YAKLAS,
    DUVAR_TAKIP,
    KOSE_DON
};
Durum mevcut_durum = DUVAR_ARA;

double hedef_mesafe = 0.0;
double tolerans = 0.0;
double min_on_mesafe = 0.6;
double max_sensor_mesafe = 8.0;
double max_acisal_hiz = 0.5;
double max_dogrusal_hiz = 0.2;

double onceki_hata = 0;
double integral_hata = 0;
ros::Time onceki_zaman;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (fabs(x) < 1.0 && fabs(y) < 1.0)
    {
        hedefe_ulasildi = true;
        ROS_INFO("Hedefe ulasildi!");
    }
}

double mesafeDuzenle(double mesafe)
{
    if (!std::isfinite(mesafe) || mesafe > max_sensor_mesafe)
    {
        return max_sensor_mesafe;
    }
    return mesafe;
}

void durumDegistir(Durum yeni_durum)
{
    if (yeni_durum != mevcut_durum)
    {
        ROS_INFO("Durum degisiyor: %d -> %d", mevcut_durum, yeni_durum);
        mevcut_durum = yeni_durum;
        onceki_hata = 0;
        integral_hata = 0;
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (hedefe_ulasildi)
    {
        geometry_msgs::Twist dur;
        dur.linear.x = 0;
        dur.angular.z = 0;
        cmd_pub.publish(dur);
        return;
    }

    int num_ranges = scan->ranges.size();

    int sag_idx = (int)((-M_PI / 2 - scan->angle_min) / scan->angle_increment);
    int on_idx = (int)((0.0 - scan->angle_min) / scan->angle_increment);

    sag_idx = std::max(0, std::min(sag_idx, num_ranges - 1));
    on_idx = std::max(0, std::min(on_idx, num_ranges - 1));

    double on_mesafe = mesafeDuzenle(scan->ranges[on_idx]);
    double sag_mesafe = mesafeDuzenle(scan->ranges[sag_idx]);

    geometry_msgs::Twist hareket_komutu;

    if (on_mesafe < min_on_mesafe)
    {
        ROS_INFO("Cok yakin engel tespit edildi: %.2f m", on_mesafe);
        hareket_komutu.linear.x = 0;
        hareket_komutu.angular.z = 0.3;
        cmd_pub.publish(hareket_komutu);
        return;
    }

    switch (mevcut_durum)
    {
    case DUVAR_ARA:
        if (sag_mesafe < max_sensor_mesafe - 0.5)
        {
            ROS_INFO("Duvar bulundu: %.2f m", sag_mesafe);
            durumDegistir(DUVARA_YAKLAS);
        }
        else
        {
            hareket_komutu.linear.x = 0.15;
            hareket_komutu.angular.z = -0.3;
            ROS_INFO("Duvar aranıyor...");
        }
        break;

    case DUVARA_YAKLAS:
        if (fabs(sag_mesafe - hedef_mesafe) < tolerans)
        {
            ROS_INFO("Duvara hizalanma tamamlandi.");
            durumDegistir(DUVAR_TAKIP);
        }
        else
        {
            double hata = hedef_mesafe - sag_mesafe;
            hareket_komutu.linear.x = 0.1;
            hareket_komutu.angular.z = 0.5 * hata;
            ROS_INFO("Duvara yaklasiliyor: hata=%.2f", hata);
        }
        break;

    case DUVAR_TAKIP:
        if (sag_mesafe >= max_sensor_mesafe - 0.5)
        {
            ROS_INFO("Duvar kaybedildi.");
            durumDegistir(DUVAR_ARA);
        }
        else
        {
            double hata = hedef_mesafe - sag_mesafe;
            double k_p = 0.7, k_d = 0.2, k_i = 0.01;

            ros::Time simdi = ros::Time::now();
            double dt = (onceki_zaman.toSec() > 0) ? (simdi - onceki_zaman).toSec() : 0.1;
            onceki_zaman = simdi;

            double turev = (dt > 0) ? (hata - onceki_hata) / dt : 0;
            onceki_hata = hata;

            if (fabs(hata) < tolerans * 2)
            {
                integral_hata += hata * dt;
                integral_hata = std::max(-2.0, std::min(2.0, integral_hata));
            }
            else
            {
                integral_hata = 0;
            }

            double kontrol = k_p * hata + k_d * turev + k_i * integral_hata;

            hareket_komutu.linear.x = max_dogrusal_hiz * (1.0 - std::min(1.0, fabs(kontrol) * 0.5));
            hareket_komutu.angular.z = kontrol;

            ROS_INFO("Duvar takip: hata=%.2f, kontrol=%.2f", hata, kontrol);
        }
        break;

    case KOSE_DON:
        if (on_mesafe > min_on_mesafe && fabs(sag_mesafe - hedef_mesafe) < tolerans * 1.5)
        {
            ROS_INFO("Kose donusu tamamlandi.");
            durumDegistir(DUVAR_TAKIP);
        }
        else
        {
            hareket_komutu.linear.x = 0.05;
            hareket_komutu.angular.z = 0.4;
            ROS_INFO("Kose donuluyor...");
        }
        break;
    }

    cmd_pub.publish(hareket_komutu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "duvar_takip");
    ros::NodeHandle nh;

    nh.param("hedef_mesafe", hedef_mesafe, 0.35);
    nh.param("tolerans", tolerans, 0.08);
    nh.param("min_on_mesafe", min_on_mesafe, 0.4);
    nh.param("max_sensor_mesafe", max_sensor_mesafe, 8.0);
    nh.param("max_acisal_hiz", max_acisal_hiz, 0.5);
    nh.param("max_dogrusal_hiz", max_dogrusal_hiz, 0.2);

    ROS_INFO("Duvar takip baslatiliyor...");

    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate rate(10);
    while (ros::ok() && !hedefe_ulasildi)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::Twist dur;
    dur.linear.x = 0;
    dur.angular.z = 0;
    cmd_pub.publish(dur);
    ROS_INFO("Gorev tamamlandi. Robot durduruldu.");

    return 0;
}