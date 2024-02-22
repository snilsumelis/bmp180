# bmp180
# BMP180 Sensörü ile Sıcaklık, Basınç ve Yükseklik Ölçme Uygulaması

Bu uygulama, Texas Instruments Tıva4c mikrodenetleyici kartı üzerinde çalışan bir örnek uygulamadır. Kod, BMP180 sensörü aracılığıyla sıcaklık, basınç ve yükseklik değerlerini ölçmek için yazılmıştır. Uygulama, TI-RTOS (Texas Instruments Gerçek Zamanlı İşletim Sistemi) kullanılarak geliştirilmiştir ve I2C iletişim protokolünü kullanarak sensör verilerini okur.

## Amaç

Bu kod, mikrodenetleyici tabanlı bir projede sıcaklık, basınç ve yükseklik gibi çevresel verilerin ölçülmesi için bir başlangıç noktası sağlar. Bu örnek, sensör verilerinin okunması, işlenmesi ve uygun bir şekilde kullanılması için temel bir yapı sunar. Ayrıca, I2C ve GPIO sürücülerinin kullanımını da örnekler.

## Özellikler

- BMP180 sensörü ile sıcaklık, basınç ve yükseklik ölçümü
- TI-RTOS kullanımı
- I2C ve GPIO sürücülerinin kullanımı

## Kurulum

1. Projeyi klonlayın:

    ```bash
    git clone https://github.com/snilsumelis/bmp180
    ```

2. CCS (Code Composer Studio) veya başka bir IDE kullanarak projeyi açın.

3. Proje bağımlılıklarını çözün. Projenin bağımlılıklarını doğru şekilde ayarladığınızdan emin olun.

4. Mikrodenetleyici kartınızı bağlayın ve projeyi hedef cihaza yükleyin.

## Kullanım

1. Uygulamayı çalıştırın.

2. Uygulama, BMP180 sensörü aracılığıyla sıcaklık, basınç ve yükseklik değerlerini ölçmeye başlayacaktır.

3. Ölçülen değerler, seri bağlantı (UART) veya başka bir arayüz aracılığıyla erişilebilir veya uygun bir ekran aracılığıyla görüntülenebilir.

