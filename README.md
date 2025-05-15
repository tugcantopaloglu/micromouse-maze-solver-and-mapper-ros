# Ödev 1
Robotlar dersi için  hazırladığım ilk ödev. Bu ödev için birden fazla geliştirme yaptım ikisini de test edebilirsiniz. İlk yaptığım geliştirmeden sonra aklıma bazı yenilikler geldiğinden ikinci geliştirmeyi de daha sonra gerçekleştirdim.


## Kurulum ve Ortamın Başlatılması
Bu ödevde micromouse maze 4 ve turtlebot3 waffle kullandım. Bunların başlatılmasını ve kullanılmasını sağlayacağız.

Ödevde aşağıda bulunan iki paket ve benim geliştirdiğim solve_maze paketi bulunuyor.
-https://gitlab.com/blm6191_2425b_tai/blm6191/turtlebot3_simulations 
-https://gitlab.com/blm6191_2425b_tai/blm6191/micromouse_maze

### Kurulum

```
git clone https://gitlab.com/blm6191_2425b_tai/members/24501118/hw1.git
```

### Ortamın Başlatılması
Ana klasöre geliyoruz.

```
cd ./hw1/robotlar_ws
```

Her ihtimale karşı kodların build'lenmesi için kodumuzu kullanıyoruz. Ve model olarak da waffle seçiyoruz.

```
export TURTLEBOT3_MODEL=waffle
catkin_make clean
catkin_make
source devel/setup.bash
```

Ardından ortamımızı başlatıyoruz.

```
roslaunch micromouse_maze micromouse_maze4.launch
```

Bu işlemi yaptığımızda gazebo'nun ve Rviz'in açılmasını beklemekteyiz. (Bu işlemler root ile yapılırsa Gazebo GUI root ile çok iyi çalışmadığından problem yaşanabilir normal bir user ile yapılması daha sağlıklı olur)

## Çözümün Gözlemlenmesi ve Çıktılar

### Çözümün Gözlemlenmesi

Çözümünü yaptığım tüm kodlar aşağıdaki lokasyonda bulunuyor bu lokasyonda 3 adet çözüm bulacaksınız. Birincisi ilk iterasyonum olan my_solver (duvara oldukça yakın giderek labirenti tamamlıyor)

```
hw1/robotlar_ws/src/solve_maze
```

İkincisi my_solver_salinimli olan ilk iterasyonu geliştirdiğim versiyon duvara daha uzaktan giderek labirenti tamamlıyor. (duvarı salınım "S" şekli yaparak takip ediyor.)

Ve son olarak my_mapper robotumuzun gezdiği alanın haritalamasını yapan kodumuz. Bu üç kodu da aşağıdaki yöntemlerle çalıştırıp nasıl çalıştığını gözlemleyebilirsiniz.

İlk çözüm
```
rosrun solve_maze my_solver
```

İkinci çözüm
```
rosrun solve_maze my_solver_salinimli
```

Map oluşturucu
```
rosrun solve_maze my_mapper
```

### Çıktılar
Labirent çalıştırıldıktan sonra ve
```
rosrun solve_maze my_solver_salinimli
```
ve
```
rosrun solve_maze my_mapper
```
kodları çalıştırıldığında robot labirenti tamamlaya çalışıp aşağıdaki gibi haritalandırmaya yapmaya başlayacaktır.

![odev1](./ciktilar/odev_1.png)

Çözümümüzü ve haritalandırmamızı bir süre çalıştırdıktan sonra robot gezerken haritalandırma da yapacağı için aşağıdaki şekilde bir görüntü ve çıktılar elde edeceğiz. (Robotun tüm kararlarını ve çıktılarını konsoldan takip edebilirsiniz.)

![img1](./ciktilar/odev1_cozum.png)

Bir süre sonra robotumuz sağ duvarı takip ederek labirenti tamamlayıp ortadaki hedef noktada kendini durduracak aşağıdaki şekilde bu pozisyonu görebiliriz.

![img2](./ciktilar/odev1_final2.png)

Çıktıları ve son haritalandırmayı da takip ettiğimizde işlemin tamamlandığına dair çıktımızı ve final haritamızı görebiliyor olacağız.

![img3](./ciktilar/odev1_final.png)

Final haritamız ise tam olarak aşağıdaki gibi olacaktır.

![img4](./ciktilar/odev1_final_harita.png)

Ve son olarak çıktılarımızda da aşağıdaki çıktıyı gördüğümüzde robot gereken yere varmış demektir.

![img5](./ciktilar/odev1_final_cmd.png)

Bu şekilde wall following ile labirenti çözmüş ve mapping işlemi yapmış olduk.