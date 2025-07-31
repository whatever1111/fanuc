#!/usr/bin/env python3
"""
Byte Order Analyzer for Welding State Messages
Helps determine if the correct byte order is being used by analyzing message values
"""

import rospy
from fanuc_driver.msg import WeldState
import struct

class ByteOrderAnalyzer:
    def __init__(self):
        self.message_count = 0
        self.analysis_results = []
        
    def analyze_byte_order(self, msg):
        """Analyze message to determine if byte order is correct"""
        self.message_count += 1
        
        # Get raw values
        raw_voltage = msg.act_voltage
        raw_current = msg.act_current
        raw_wire_spd = msg.act_wire_spd
        
        # Normal interpretation (as received)
        voltage_normal = raw_voltage * 100.0 / 32767.0
        current_normal = raw_current * 1000.0 / 32767.0
        wire_speed_normal = raw_wire_spd * 40.0 / 32767.0
        
        # Byte-swapped interpretation
        voltage_swapped = self.swap_bytes(raw_voltage) * 100.0 / 32767.0
        current_swapped = self.swap_bytes(raw_current) * 1000.0 / 32767.0
        wire_speed_swapped = self.swap_bytes(raw_wire_spd) * 40.0 / 32767.0
        
        # Analyze which interpretation makes more sense
        normal_score = self.score_values(voltage_normal, current_normal, wire_speed_normal)
        swapped_score = self.score_values(voltage_swapped, current_swapped, wire_speed_swapped)
        
        result = {
            'msg_num': self.message_count,
            'raw_values': (raw_voltage, raw_current, raw_wire_spd),
            'normal': {
                'voltage': voltage_normal,
                'current': current_normal,
                'wire_speed': wire_speed_normal,
                'score': normal_score
            },
            'swapped': {
                'voltage': voltage_swapped,
                'current': current_swapped,
                'wire_speed': wire_speed_swapped,
                'score': swapped_score
            },
            'recommendation': 'normal' if normal_score > swapped_score else 'swapped'
        }
        
        self.analysis_results.append(result)
        
        # Print analysis for this message
        self.print_message_analysis(result)
        
        # Print summary every 10 messages
        if self.message_count % 10 == 0:
            self.print_summary()
    
    def swap_bytes(self, value):
        """Swap bytes in a 16-bit signed integer"""
        # Convert to unsigned, swap bytes, convert back to signed
        if value < 0:
            value = 65536 + value  # Convert to unsigned representation
        
        # Swap bytes
        swapped = ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8)
        
        # Convert back to signed
        if swapped > 32767:
            swapped = swapped - 65536
            
        return swapped
    
    def score_values(self, voltage, current, wire_speed):
        """Score how reasonable the values are for welding"""
        score = 0
        
        # Voltage scoring (typical range: 10-50V)
        if 10 <= voltage <= 50:
            score += 3
        elif 5 <= voltage <= 60:
            score += 2
        elif 0 <= voltage <= 100:
            score += 1
        
        # Current scoring (typical range: 50-500A)
        if 50 <= current <= 500:
            score += 3
        elif 20 <= current <= 600:
            score += 2
        elif 0 <= current <= 1000:
            score += 1
        
        # Wire speed scoring (typical range: 2-20 m/min)
        if 2 <= wire_speed <= 20:
            score += 3
        elif 1 <= wire_speed <= 25:
            score += 2
        elif 0 <= wire_speed <= 40:
            score += 1
        
        # Penalty for negative values (shouldn't happen in welding)
        if voltage < 0:
            score -= 2
        if current < 0:
            score -= 2
        if wire_speed < 0:
            score -= 2
        
        # Penalty for extremely large values
        if abs(voltage) > 1000:
            score -= 3
        if abs(current) > 10000:
            score -= 3
        if abs(wire_speed) > 1000:
            score -= 3
        
        return score
    
    def print_message_analysis(self, result):
        """Print analysis for a single message"""
        print(f"\n--- Message {result['msg_num']} Analysis ---")
        print(f"Raw values: V={result['raw_values'][0]}, I={result['raw_values'][1]}, W={result['raw_values'][2]}")
        
        print(f"Normal interpretation:")
        print(f"  Voltage: {result['normal']['voltage']:.1f}V")
        print(f"  Current: {result['normal']['current']:.0f}A")
        print(f"  Wire Speed: {result['normal']['wire_speed']:.1f}m/min")
        print(f"  Score: {result['normal']['score']}")
        
        print(f"Swapped interpretation:")
        print(f"  Voltage: {result['swapped']['voltage']:.1f}V")
        print(f"  Current: {result['swapped']['current']:.0f}A")
        print(f"  Wire Speed: {result['swapped']['wire_speed']:.1f}m/min")
        print(f"  Score: {result['swapped']['score']}")
        
        if result['recommendation'] == 'normal':
            print("✓ Normal byte order appears correct")
        else:
            print("⚠ Swapped byte order may be needed")
    
    def print_summary(self):
        """Print summary of all analyzed messages"""
        if not self.analysis_results:
            return
        
        normal_wins = sum(1 for r in self.analysis_results if r['recommendation'] == 'normal')
        swapped_wins = sum(1 for r in self.analysis_results if r['recommendation'] == 'swapped')
        
        print("\n" + "=" * 50)
        print(f"BYTE ORDER ANALYSIS SUMMARY ({len(self.analysis_results)} messages)")
        print("=" * 50)
        print(f"Normal byte order recommended: {normal_wins} times")
        print(f"Swapped byte order recommended: {swapped_wins} times")
        
        if normal_wins > swapped_wins:
            print("✓ RECOMMENDATION: Use normal byte order (use_bswap:=false)")
        elif swapped_wins > normal_wins:
            print("⚠ RECOMMENDATION: Use swapped byte order (use_bswap:=true)")
        else:
            print("? INCONCLUSIVE: Equal scores for both byte orders")
        
        # Calculate average scores
        avg_normal_score = sum(r['normal']['score'] for r in self.analysis_results) / len(self.analysis_results)
        avg_swapped_score = sum(r['swapped']['score'] for r in self.analysis_results) / len(self.analysis_results)
        
        print(f"Average normal score: {avg_normal_score:.1f}")
        print(f"Average swapped score: {avg_swapped_score:.1f}")
        
        # Show typical value ranges
        if self.analysis_results:
            last_result = self.analysis_results[-1]
            print(f"\nLatest values (normal): V={last_result['normal']['voltage']:.1f}V, "
                  f"I={last_result['normal']['current']:.0f}A, W={last_result['normal']['wire_speed']:.1f}m/min")
            print(f"Latest values (swapped): V={last_result['swapped']['voltage']:.1f}V, "
                  f"I={last_result['swapped']['current']:.0f}A, W={last_result['swapped']['wire_speed']:.1f}m/min")
        
        print("=" * 50)
    
    def print_final_report(self):
        """Print final analysis report"""
        if not self.analysis_results:
            print("No messages analyzed")
            return
        
        normal_wins = sum(1 for r in self.analysis_results if r['recommendation'] == 'normal')
        swapped_wins = sum(1 for r in self.analysis_results if r['recommendation'] == 'swapped')
        
        print("\n" + "=" * 60)
        print("FINAL BYTE ORDER ANALYSIS REPORT")
        print("=" * 60)
        print(f"Total messages analyzed: {len(self.analysis_results)}")
        print(f"Normal byte order recommended: {normal_wins} times ({normal_wins/len(self.analysis_results)*100:.1f}%)")
        print(f"Swapped byte order recommended: {swapped_wins} times ({swapped_wins/len(self.analysis_results)*100:.1f}%)")
        
        if normal_wins > swapped_wins * 1.5:
            print("\n✓ STRONG RECOMMENDATION: Use normal byte order")
            print("  Launch with: use_bswap:=false")
        elif swapped_wins > normal_wins * 1.5:
            print("\n⚠ STRONG RECOMMENDATION: Use swapped byte order")
            print("  Launch with: use_bswap:=true")
        elif normal_wins > swapped_wins:
            print("\n✓ WEAK RECOMMENDATION: Use normal byte order")
            print("  Launch with: use_bswap:=false")
        elif swapped_wins > normal_wins:
            print("\n⚠ WEAK RECOMMENDATION: Use swapped byte order")
            print("  Launch with: use_bswap:=true")
        else:
            print("\n? INCONCLUSIVE: Unable to determine correct byte order")
            print("  Try both settings and see which produces reasonable values")
        
        print("=" * 60)

def main():
    rospy.init_node('byte_order_analyzer', log_level=rospy.INFO)
    
    analyzer = ByteOrderAnalyzer()
    
    rospy.loginfo("Byte Order Analyzer started")
    rospy.loginfo("Analyzing /weld_state messages to determine correct byte order...")
    rospy.loginfo("Press Ctrl+C to stop and see final report")
    
    def callback(msg):
        analyzer.analyze_byte_order(msg)
    
    try:
        sub = rospy.Subscriber('/weld_state', WeldState, callback)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Analysis stopped by user")
    finally:
        analyzer.print_final_report()

if __name__ == '__main__':
    main() 