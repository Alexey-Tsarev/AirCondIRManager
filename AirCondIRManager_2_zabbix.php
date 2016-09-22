<?php
// Cfg
$configUrl = 'http://10.1.2.56';
$configHostName = 'laser-manager';
$configZabbixSenderCmd = 'zabbix_sender';
$configZabbixHost = '127.0.0.1';

$k[] = 'temp';
$k[] = 'tempMin';
$k[] = 'tempMax';
$k[] = 'tempMaxAlarm';
$k[] = 'tempGrowStatus';
$k[] = 'alarmStatus';
$k[] = 'status';
$k[] = 'freeHeap';
$k[] = 'uptime';
// End Cfg

$jsonRaw = file_get_contents($configUrl);

if ($jsonRaw === false) {
    echo "Failed to get: $configUrl";
} else {
    $json = json_decode($jsonRaw, true);

    if ($json === NULL) {
        echo "Failed to decode json";
        exit(1);
    } else {
        $kv = "";
        $count_k = count($k);

        for ($i = 0; $i < $count_k; $i++) {
            $kv .= $configHostName . " " . $k[$i] . " " . $json[$k[$i]];

            if ($i != $count_k - 1)
                $kv .= "\n";
        }

        $cmd = "echo \"" . $kv . "\" | " . $configZabbixSenderCmd . " -z " . $configZabbixHost . " -i -";
        echo "Run: $cmd\n";
        exec($cmd, $output, $ret);

        echo "Output: ";
        print_r($output);

        if ($ret !== 0) {
            echo "Failed with the exit code: $ret\n";
            exit($ret);
        }
    }
}
